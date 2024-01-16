#include <ros/ros.h>

#include "graph.h"

namespace uware
{
  Graph::Graph() {}

  void Graph::optimize(vector<PoseInfo>& poses, vector<EdgeInfo>& edges)
  {
    vertex_names_.clear();

    // Initialize the g2o graph optimizer
    g2o::SparseOptimizer graph_optimizer;
    g2o::BlockSolverX::LinearSolverType * linear_solver_ptr;
    linear_solver_ptr = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linear_solver_ptr);
    g2o::OptimizationAlgorithmLevenberg * solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    graph_optimizer.setAlgorithm(solver);

    // Add vertices
    for (uint i=0; i<poses.size(); i++)
    {
      // Convert pose for graph
      Eigen::Isometry3d vertex_pose = Utils::tfToIsometry(poses[i].pose);

      // Build the vertex
      g2o::VertexSE3* cur_vertex = new g2o::VertexSE3();
      cur_vertex->setId((int)i);
      cur_vertex->setEstimate(vertex_pose);
      if ((int)i == 0)
      {
        // First time, no edges.
        cur_vertex->setFixed(true);
      }
      graph_optimizer.addVertex(cur_vertex);

      vertex_names_.push_back(make_pair(i,poses[i].name));
    }

    // Add edges
    for (uint i=0; i<edges.size(); i++)
    {
      int a = searchIdx(edges[i].name_a);
      int b = searchIdx(edges[i].name_b);
      if (a<0)
        ROS_ERROR_STREAM("[Reconstruction]: Bad vertex index for name " << edges[i].name_a);
      if (b<0)
        ROS_ERROR_STREAM("[Reconstruction]: Bad vertex index for name " << edges[i].name_b);
      g2o::VertexSE3* v_a = dynamic_cast<g2o::VertexSE3*>(graph_optimizer.vertices()[a]);
      g2o::VertexSE3* v_b = dynamic_cast<g2o::VertexSE3*>(graph_optimizer.vertices()[b]);

      // Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity() * (double)sigma;

      // Add the new edge to graph
      g2o::EdgeSE3* e = new g2o::EdgeSE3();
      Eigen::Isometry3d t = Utils::tfToIsometry(edges[i].edge);
      e->setVertex(0, v_a);
      e->setVertex(1, v_b);
      e->setMeasurement(t);
      // e->setInformation(information);
      graph_optimizer.addEdge(e);
    }

    // Optimize!
    graph_optimizer.initializeOptimization();
    graph_optimizer.optimize(20);

    // Return the optimized poses
    vector<PoseInfo> new_poses;
    for (uint i=0; i<graph_optimizer.vertices().size(); i++)
    {
      g2o::VertexSE3* v =  dynamic_cast<g2o::VertexSE3*>(graph_optimizer.vertices()[i]);
      tf::Transform pose = Utils::isometryToTf(v->estimate());
      PoseInfo p(poses[i].name, pose);
      new_poses.push_back(p);
    }

    // Overwrite output
    poses = new_poses;
  }

  int Graph::searchIdx(string name)
  {
    int out = -1;
    for (uint i=0; i<vertex_names_.size(); i++)
    {
      if (vertex_names_[i].second == name)
      {
        out = vertex_names_[i].first;
        break;
      }
    }
    return out;
  }


} // namespace