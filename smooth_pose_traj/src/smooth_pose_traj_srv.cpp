#include <ros/ros.h>
#include <boost/format.hpp>
#include <mutex>
#include <thread>
#include <smooth_pose_traj/SmoothPoseTrajectory.h> // the service
#include <smooth_pose_traj/smooth_pose_traj.hpp> // the .hpp for this cpp file

static const std::string POSE_TRAJ_SMOOTHER = "pose_trajectory_smoother";
namespace PoseTrajSmooth
{

class PoseTrajSmoother
{
public:
  PoseTrajSmoother(ros::NodeHandle nh, std::string server_name):
    nh_(nh)
  {
    smooth_pose_traj_srv_ = nh_.advertiseService(POSE_TRAJ_SMOOTHER, &PoseTrajSmoother::smoothPoseTrajCB, this);
  }

  ~PoseTrajSmoother()
  {

  }

  bool smoothPoseTrajCB(smooth_pose_traj::SmoothPoseTrajectory::Request& req, smooth_pose_traj::SmoothPoseTrajectory::Response& res)
  {
    SmoothPoseTraj::SmoothPoseTraj SPT(req.input_poses, req.point_spacing);
    return(SPT.process(res.output_poses, req.point_spacing));
  }
  ros::ServiceServer smooth_pose_traj_srv_;
  ros::NodeHandle nh_;
};

} // end SmoothPoseTraj Namespace
int main(int argc,char** argv)
{
  ros::init(argc,argv,"pose_trajectory_smoother");
  ros::AsyncSpinner spinner(4);
  ros::NodeHandle nh;
  spinner.start();
  PoseTrajSmooth::PoseTrajSmoother PTS(nh, POSE_TRAJ_SMOOTHER);
  ros::waitForShutdown();
  return 0;
}
