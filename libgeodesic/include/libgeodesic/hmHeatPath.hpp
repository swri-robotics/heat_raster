/**
 * @file
 * @author Chris Lewis <clewis@swri.org>
 * @version 1.0
 *
 * @section LICENSE
 *
 * Copyright 2020 Chris Lewis. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE FREEBSD PROJECT OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the author and should not be interpreted as representing official policies,
 * either expressed or implied, of any other person or institution.
 *
 */

#ifndef LIBGEODESIC_HMDPath_H
#define LIBGEODESIC_HMDPath_H

#include <boost/property_map/property_map.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/pending/indirect_cmp.hpp>
#include <boost/range/irange.hpp>

#include <stddef.h>
#include <stdlib.h>

#include <eigen3/Eigen/src/misc/lapacke.h>
#include <eigen3/Eigen/Eigen>

extern "C" {
#include "libgeodesic/hmTriDistance.h"
#include "libgeodesic/hmVectorSizeT.h"
#include "libgeodesic/hmUtility.h"
}

static const size_t hmVchainDefaultStorage = 16;

/** \brief representation used to create tool paths using the distance function on a surface
 *
 * All connected vertices at specified distance intervals from a source form the basis for a path
 *
 */
typedef struct vchain
{
  size_t size;
  size_t storage;
  hmVectorSizeT* entries;
} vchain;

void vchainInitialize(vchain* VCS);
void vchainDestroy(vchain* VCS);
void vchainResize(vchain* VCS, size_t size);
void vchainPushBack(vchain* VCS, hmVectorSizeT* vchain);
int max_d_exclude_v(std::vector<double>& d, double v_exclude);
void move_vertices(hmVectorSizeT* from,
                   hmVectorSizeT* to);  // move one list of vertices to another, leaving from vector empty
void showVectorSizeT(hmVectorSizeT* V, char* msg);
void showNeighbors(const hmVectorPairSizeTDouble* VN, char* msg);
void saveVertices(hmVectorSizeT* VI, double* vertices, char* file_name, char* vector_name);
double vertex_distance(double* vertices, int v1_index, int v2_index);

//-------------------------------------------------------------------------------------------
/// Cycle detector for an undirected graph
/**
Passed by value as visitor to \c boost::undirected_dfs()

See http://www.boost.org/doc/libs/1_58_0/libs/graph/doc/undirected_dfs.html
*/

class hmTriHeatPaths
{
public:
  typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;
  typedef boost::property<boost::vertex_color_t, boost::default_color_type> VertexColorProperty;
  typedef boost::adjacency_list<boost::listS,
                                boost::vecS,
                                boost::undirectedS,
                                boost::disallow_parallel_edge_tag,
                                EdgeWeightProperty,
                                VertexColorProperty>
      VChainGraph;
  typedef boost::property_map<VChainGraph, boost::vertex_index_t>::type IndexMap;
  typedef boost::graph_traits<VChainGraph>::edge_iterator edge_itr;
  typedef boost::graph_traits<VChainGraph>::edge_descriptor edge_desc;
  typedef boost::graph_traits<VChainGraph>::vertex_descriptor vertex_des;
  typedef boost::property_map<VChainGraph, boost::vertex_index_t> Vertex_id;

  typedef struct Pose
  {
    double qx, qy, qz, qw, x, y, z;
  } Pose;

  /** \brief default constructor */
  hmTriHeatPaths();

  /** \brief constructor */
  hmTriHeatPaths(hmTriDistance* distance, double raster_spacing);
  //    hmTriHeatPaths(hmTriDistance *distance, int num_levels,  hmVectorSizeT* S);

  /** \brief default destructor */
  ~hmTriHeatPaths() { free(inband_vertex_lists_); };

  // find all vertices in each band
  void compute_inband_verticies(hmTriDistance* distance, const std::vector<int>& sources);

  // find all vertex chains in each band (all edge connected vertices in same band)
  void compute_vchains(hmTriDistance* distance);

  // find all chains in each band
  void compute_vchain_graphs(hmTriDistance* distance);

  // find distances between first and all other vertices
  bool compute_dijkstras_on_graph(vertex_des& s, VChainGraph& G, std::vector<double>& d, std::vector<vertex_des>& p);

  // find paths in each Gs_[i]
  void compute_depth_first_paths(hmTriDistance* distance);

  // copies path j at tail of path i, j is reversed if invert_j is true
  void concatenate_paths(int path_i, int path_j, bool invert_j);

  // concatenates paths that end near one another's start or stop
  void connect_paths(hmTriDistance* distance);

  // adds the sources as a vertex sequence
  void add_sources_to_sequences(hmTriDistance* distance, const std::vector<int>& source_indices);
  void add_sources_to_sequences(hmTriDistance* distance, const hmVectorSizeT* sources);

  // convert paths to Pose Array
  void compute_pose_arrays(hmTriDistance* distance);

  // reduce graph excluding vertices close to these
  void reduce_graph_exclude_near_verts(hmTriDistance* distance,
                                       VChainGraph& G_in,
                                       VChainGraph& G_out,
                                       const std::vector<vertex_des>& V_e);

  // find a path from a reduced form of each Gs_[i]
  std::vector<hmTriHeatPaths::vertex_des> find_path(hmTriDistance* distance, VChainGraph& G);

  // compute everything
  void compute(hmTriDistance* distance, const std::vector<int>& source_indices);

  // remove a vchain from vertex_list
  // a vchain starts with the first vertex in vertex_list, and finds all connected neighbors that are in the same band
  char extract_vchain(hmTriDistance* distance, hmVectorSizeT* vertex_list, double band, hmVectorSizeT* vchain);

  // build a graph using a vchain,  all edge connected vertices in same distance band
  void build_vchain_graph(hmTriDistance* distance, hmVectorSizeT* vchain, VChainGraph& G);

  // write the paths found using bfs
  void octave_output_paths(std::string filename, hmTriDistance* distance);

  // write source set as a path for display
  void octave_output_sources(std::string filename, hmTriDistance* distance, hmVectorSizeT* sourceSets);

  struct CycleDetector : public boost::dfs_visitor<>
  {
  public:
    CycleDetector(){};
    void Clear() { back_edges_.clear(); }
    bool cycleDetected() const { return !back_edges_.empty(); }
    void back_edge(VChainGraph::edge_descriptor e, const VChainGraph& g)  // is invoked on the back edges in the graph.
    {
      back_edges_.push_back(e);
    }
  };

  struct PathGen : public boost::bfs_visitor<>
  {
  public:
    PathGen(hmTriDistance* distance)
    {
      vertices = distance->surface->vertices;
      ave_edge_length = sqrt(distance->time);
    };
    void Clear()
    {
      path_list_.clear();
      discard_list_.clear();
    }
    bool path_found() const { return path_list_.size() > 1; }

    // find closest vertex in path_list to the given vertex
    std::list<vertex_des>::iterator find_closest_point(std::list<vertex_des>& list, int v_index, double& cd);

    // for boost::breadth_first_search() invoked on first discovery of every vertex
    void discover_vertex(vertex_des& v, const VChainGraph& G);

    double* vertices;

    /** \brief length of average edge */
    double ave_edge_length;

  };  // end of PathGen structure
private:
  // check to see if a vertex distance is within the desired distance band
  char is_inband(hmTriDistance* distance, size_t vertex_index, double band);

  // check to see if a particular index is NOT in the vector
  char notInVectorSizeT(hmVectorSizeT* list, size_t index);

  // finds the normal at a vertex by doing a curve fit to its neighbors
  void plane_fit(hmTriDistance* distance, size_t vertex_index, Eigen::Vector3d& normal_vec);

  // finds the averge face normal at this vertex
  void face_normal(hmTriDistance* distance, size_t vertex_index, Eigen::Vector3d& normal_vec);

  // finds the face normal using only one face from this vertex
  void one_face_normal(hmTriDistance* distance, size_t vertex_index, Eigen::Vector3d& normal_vec);

  // from one, list remove any matches form the other list
  void remove_matching_vertices(hmVectorSizeT* to_match, hmVectorSizeT* vertex_list);

public:
  // connected vertices all within each distance band
  vchain vcs_;

  // Graphs formed from each vchain
  std::vector<VChainGraph> Gs_;

  /** \brief list of vertices (indexes) found to be at each distance band */
  hmVectorSizeT* vertex_lists_;

  /** \brief vectors orthogonal to the polygon at a vertex **/
  double* vertexNormals_;  // compute normals at every vertex, why not?

  /** \brief the number of distance levels used to create the paths, not necessarily same as number of path segments */
  int num_levels_;

  /** \brief the tolerance in distance to be in a path segment k*delta - epsilon < distance < k*delta + epsilon */
  double epsilon_;

  /** \brief the distance between each heat band determined by max_distance, and num_levels */
  double delta_;

  /** \brief sets of in-band vertices for each band */
  hmVectorSizeT* inband_vertex_lists_;

  /** \brief largest distance in the distance field*/
  double max_distance_;

  /** \brief paths found using bfs on vchains */
  std::vector<std::vector<vertex_des>> vertex_sequences_;

  /** \brief poses at each vertex along a path */
  std::vector<std::vector<Pose>> pose_arrays_;

  //  /** \brief the sources are the vertices set to a constant temp for distance computation */
  //  hmVectorSizeT* S_;

  /** \brief the back edges found using depth first search on the VChainGraphs */
  static std::vector<hmTriHeatPaths::VChainGraph::edge_descriptor> back_edges_;
  static std::list<vertex_des> path_list_;
  static std::list<vertex_des> discard_list_;
  static std::map<int, size_t> vmap_;   // create a map between vertices in Gs_[i] and a new graph GI
  static std::map<int, size_t> ivmap_;  // create a map between vertices in Gs_[i] and a new graph GI
};

#endif /* LIBGEODESIC_HMDPath_H */
