#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/format.hpp>
#include <mutex>
#include <thread>
#include <heat_msgs/GenerateHeatToolPathsAction.h>
#include <heat_ros/heat_surface_planner.hpp>
#include <smooth_pose_traj/SmoothPoseTrajectory.h>  // the service to smooth and sample a trajectory
static const std::string GENERATE_TOOL_PATHS_ACTION = "generate_heat_tool_paths";
static const std::string SMOOTH_TOOL_PATHS_SERVICE = "pose_trajectory_smoother";

namespace heat_path_gen
{
using GenPathActionServer = actionlib::SimpleActionServer<heat_msgs::GenerateHeatToolPathsAction>;

class HeatServer
{
public:
  HeatServer(ros::NodeHandle nh, std::string action_name) : nh_(nh), as_(nh, action_name, false)
  {
    path_smooth_client_ = nh_.serviceClient<smooth_pose_traj::SmoothPoseTrajectory>(SMOOTH_TOOL_PATHS_SERVICE);
  }

  ~HeatServer() {}

  void start()
  {
    as_.registerGoalCallback(boost::bind(&HeatServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&HeatServer::preemptCB, this));
    as_.start();
  }

protected:
  /** goal has:
   *  HeatMeshNSource[] mesh_n_s which has both a mesh and a set of source vertices
   *  HeatRasterGeneratorConfig[] path_configs
   *  bool proceed_on_failure  (a currently unused parameter)
   *  result has:
   *  HeatToolPath[] tool_raster_paths
   *  bool tool_path_validities        these indicate which if any of the paths provided are invalid. Not sure why an
   *invalid path is returned?
   **/
  void runHeatPathGeneration(const GenPathActionServer::GoalConstPtr goal)
  {
    ROS_INFO("Starting heat path generation");
    heat_msgs::GenerateHeatToolPathsResult result;
    // result has:
    // bool success
    // HeatToolPath[] tool_raster_paths	# The resulting raster paths
    //     geometry_msgs/PoseArray[] paths
    // bool[] tool_path_validities		# True when the tool path in 'tool_raster_paths' with the same index is valid.

    // start lock
    {
      std::lock_guard<std::mutex> lock(goal_process_mutex_);

      // validate data
      if (goal->surface_meshes.empty())
      {
        std::string err_msg = "No surfaces were received";
        ROS_ERROR_STREAM(err_msg);
        as_.setAborted(result, err_msg);
        return;
      }

      // verify configs
      std::vector<heat_msgs::HeatRasterGeneratorConfig> heat_configs;
      if (goal->path_configs.empty())
      {
        ROS_WARN("Path configuration array is empty, using default values");
        heat_configs.resize(goal->surface_meshes.size(), toHeatMsg(heat::HeatSurfacePlanner::getDefaultConfig()));
      }
      else
      {
        heat_configs.assign(goal->path_configs.begin(), goal->path_configs.end());
      }

      // veryfy number of configs & surfaces match
      if (heat_configs.size() != goal->surface_meshes.size())
      {
        std::string err_msg = "Surface meshes and path configs array have unequal sizes";
        ROS_ERROR_STREAM(err_msg);
        as_.setAborted(result, err_msg);
        return;
      }
    }  // end lock

    // call planner on each mesh & source list
    const std::size_t num_meshes = goal->surface_meshes.size();
    std::vector<bool> validities;
    using ToolPath = std::vector<geometry_msgs::PoseArray>;
    result.tool_path_validities.resize(num_meshes, false);
    result.tool_raster_paths.resize(num_meshes);
    for (std::size_t i = 0; i < goal->surface_meshes.size(); i++)  // for each mesh & source list
    {
      {  // start lock
        std::lock_guard<std::mutex> lock(goal_process_mutex_);
        if (as_.isPreemptRequested())
        {
          ROS_WARN("Canceling Tool Path Generation Request");
          break;
        }
      }  // end lock

      // plan heat paths
      heat::ProcessConfig config = toProcessConfig(goal->path_configs[i]);
      heat::HeatSurfacePlanner path_gen(config);
      std::vector<geometry_msgs::PoseArray> heat_tool_paths;
      path_gen.planPaths(goal->surface_meshes[i], goal->sources[0].source_indices, heat_tool_paths);

      // smooth and resample paths at perscribed point spacing
      smooth_pose_traj::SmoothPoseTrajectory::Request smooth_req;
      smooth_pose_traj::SmoothPoseTrajectory::Response smooth_res;
      std::vector<geometry_msgs::PoseArray> smoothed_tool_paths;
      smooth_req.point_spacing = config.point_spacing;
      for (int j = 0; j < heat_tool_paths.size(); j++)
      {
        smooth_req.input_poses.poses.assign(heat_tool_paths[j].poses.begin(), heat_tool_paths[j].poses.end());
        if (!path_smooth_client_.call(smooth_req, smooth_res))
        {
          ROS_ERROR("path_smooth_client call failed");
        }
        else
        {
          geometry_msgs::PoseArray PA;
          PA.poses.assign(smooth_res.output_poses.poses.begin(), smooth_res.output_poses.poses.end());
          smoothed_tool_paths.push_back(PA);
        }
      }  // end smooth and re-sampling

      if (smoothed_tool_paths.size() > 0)
      {
        heat_msgs::HeatToolPath trp;
        for (int j = 0; j < smoothed_tool_paths.size(); j++)  // copy tool paths into the message for return
        {
          geometry_msgs::PoseArray path;
          path.poses.assign(smoothed_tool_paths[j].poses.begin(), smoothed_tool_paths[j].poses.end());
          trp.paths.push_back(path);
        }
        // need to fill in trp.header since its sent to the result, but with what??
        result.tool_raster_paths[i] = move(trp);
        result.tool_path_validities[i] = true;
        ROS_INFO("Surface %ld processed with %ld paths", i, result.tool_raster_paths[i].paths.size());
      }  // end tool paths size was non-zero for this surface
      else
      {
        ROS_ERROR("Path planning on surface %ld failed", i);
        if (!goal->proceed_on_failure)
        {
          break;
        }
      }  // end tool path size was zero for this surface
    }    // end for each surface

    // if any paths are invalid report failure in result
    result.success = std::any_of(
        result.tool_path_validities.begin(), result.tool_path_validities.end(), [](const bool& b) { return b; });

    // return result to action
    {  // start lock
      std::lock_guard<std::mutex> lock(goal_process_mutex_);

      if (result.success)
      {
        ROS_INFO("Heat method generated result for %ld meshes", result.tool_raster_paths.size());
        for (size_t jj = 0; jj < result.tool_raster_paths.size(); jj++)
        {
          ROS_INFO("Mesh %ld has %ld paths", jj, result.tool_raster_paths[jj].paths.size());
          for (size_t kk = 0; kk < result.tool_raster_paths[jj].paths.size(); kk++)
          {
            ROS_INFO(
                "Mesh %ld path %ld has %ld waypoints", jj, kk, result.tool_raster_paths[jj].paths[kk].poses.size());
          }
        }

        as_.setSucceeded(result);
      }
      else
      {
        as_.setAborted(result);
      }
    }  // end lock
  }

  void goalCB()
  {
    GenPathActionServer::GoalConstPtr goal;
    {
      std::lock_guard<std::mutex> lock(goal_process_mutex_);

      if (as_.isActive())
      {
        ROS_ERROR("Currently Processing a HEAT request, rejecting");
        return;
      }

      goal = as_.acceptNewGoal();
      if (goal == nullptr)
      {
        std::string err_msg = "Goal ptr received is invalid";
        ROS_ERROR_STREAM(err_msg);
        heat_msgs::GenerateHeatToolPathsResult result;
        as_.setAborted(result, err_msg);
        return;
      }
    }

    runHeatPathGeneration(goal);
  }

  void preemptCB()
  {
    std::lock_guard<std::mutex> lock(goal_process_mutex_);
    as_.setPreempted();
  }

  heat_msgs::HeatRasterGeneratorConfig toHeatMsg(const heat::ProcessConfig& tool_config)
  {
    heat_msgs::HeatRasterGeneratorConfig heat_config_msg;
    heat_config_msg.point_spacing = tool_config.point_spacing;
    heat_config_msg.raster_spacing = tool_config.raster_spacing;
    heat_config_msg.tool_offset = tool_config.tool_offset;
    heat_config_msg.min_hole_size = tool_config.min_hole_size;
    heat_config_msg.min_segment_size = tool_config.min_segment_size;
    heat_config_msg.generate_extra_rasters = tool_config.generate_extra_rasters;
    heat_config_msg.raster_rot_offset = tool_config.raster_rot_offset;
    return std::move(heat_config_msg);
  }

  heat::ProcessConfig toProcessConfig(const heat_msgs::HeatRasterGeneratorConfig& heat_config_msg)
  {
    heat::ProcessConfig tool_config;
    tool_config.point_spacing = heat_config_msg.point_spacing;
    tool_config.raster_spacing = heat_config_msg.raster_spacing;
    tool_config.tool_offset = heat_config_msg.tool_offset;
    tool_config.min_hole_size = heat_config_msg.min_hole_size;
    tool_config.min_segment_size = heat_config_msg.min_segment_size;
    tool_config.generate_extra_rasters = heat_config_msg.generate_extra_rasters;
    tool_config.raster_rot_offset = heat_config_msg.raster_rot_offset;
    return std::move(tool_config);
  }

  ros::NodeHandle nh_;
  GenPathActionServer as_;
  ros::ServiceClient path_smooth_client_;
  std::mutex goal_process_mutex_;
};

}  // namespace heat_path_gen

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_generator");
  ros::AsyncSpinner spinner(4);
  ros::NodeHandle nh;
  spinner.start();
  heat_path_gen::HeatServer heat_server(nh, GENERATE_TOOL_PATHS_ACTION);
  heat_server.start();
  ros::waitForShutdown();
  return 0;
}
