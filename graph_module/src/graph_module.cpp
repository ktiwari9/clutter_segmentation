/*********************************************************************
 *
 *  Copyright (c) 2012, Computational Learning and Motor Control Laboratory
 *  University of Southern California
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************
/**
 * \author Bharath Sankaran
 *
 * @b computes connectivity graph for static segmented connected components
 */

#include "graph_module/graph_module.hpp"
#include <graph_module/EGraph.h>
#include <graph_module/VGraph.h>
#include <list>
#include <iterator>

namespace graph {

ros_graph::ros_graph(int number_of_vertices):
	    graph_(){

  number_of_vertices_ = number_of_vertices;
}

ros_graph::ros_graph():
	    graph_(){
}

ros_graph::~ros_graph(){}

bool ros_graph::buildGraph(graph_module::EGraph in_graph_msg){

//  ROS_INFO("Okay so we entered the message");
  if(in_graph_msg.edges.size() == 0)
    return false;
  else{
    graph_.clear();
    for(int i=0; i<in_graph_msg.edges.size(); i++)
      addEdge(in_graph_msg.edges[i].start.index,in_graph_msg.edges[i].start.point.x,in_graph_msg.edges[i].start.point.y,
              in_graph_msg.edges[i].end.index,in_graph_msg.edges[i].end.point.x,in_graph_msg.edges[i].end.point.y,
              in_graph_msg.edges[i].weight);

     // Now compute the adjacency matrix
//    ROS_INFO("Entering adjacency matrix computation loop");
    adjacency_matrix_ = computeAdjacencyMatrix();
    number_of_vertices_ = adjacency_matrix_.rows();
    return true;
  }

}

bool ros_graph::buildGraph(graph_module::VGraph in_graph_msg){

  return false;
}

graph_module::EGraph ros_graph::convertToEGraphMsg(){

  ROS_INFO("Converting to E graph Message");
  graph_module::EGraph out_msg;
  for(IGraph_it iter_=graph_.begin(); iter_!=graph_.end(); iter_++){

    graph_module::Edge edge;

    // initializing points
    edge.start.point.x = iter_->edge_.first.x_;
    edge.start.point.y = iter_->edge_.first.y_;
    edge.start.index = iter_->edge_.first.index_;

    edge.end.point.x = iter_->edge_.second.x_;
    edge.end.point.y = iter_->edge_.second.y_;
    edge.end.index = iter_->edge_.second.index_;

    edge.weight = iter_->weight_; // Random initialization now

    out_msg.edges.push_back(edge);

  }

  return out_msg;

}

void ros_graph::addEdge(Vertex_ros v1, Vertex_ros v2, double weight){

  Edge_ros e;
  e.edge_ = make_pair(v1,v2);
  e.weight_ = weight;

  graph_.insert(e);
}

void ros_graph::addEdge(int node_1, double centroid_x1, double centroid_y1,
                        int node_2, double centroid_x2, double centroid_y2, double weight){

  Vertex_ros v1, v2;
  v1.index_ = node_1;
  v1.x_ = centroid_x1;
  v1.y_ = centroid_y1;
  v2.index_ = node_2;
  v2.x_ = centroid_x2;
  v2.y_ = centroid_y2;

  Edge_ros e;
  e.edge_ = make_pair(v1,v2);
  e.weight_ = weight;

  graph_.insert(e);
}

bool ros_graph::findVertex(Vertex_ros v){

  for(IGraph_it iter_ = graph_.begin(); iter_ != graph_.end(); iter_++)
    if(iter_->edge_.first.index_ == v.index_ || iter_->edge_.second.index_ == v.index_ )
      return true;

  return false;

}

bool ros_graph::findVertex(int index){

  for(IGraph_it iter_ = graph_.begin(); iter_ != graph_.end(); iter_++)
    if(iter_->edge_.first.index_ == index || iter_->edge_.second.index_ == index)
      return true;

  return false;

}

bool ros_graph::findEdge(Edge_ros e){

  IGraph_it iter_ = graph_.find(e);

  if(iter_ == graph_.end())
    return false;
  else
    return true;
}

bool ros_graph::findEdge(Vertex_ros v1, Vertex_ros v2){

  Edge_ros e;
  e.edge_ = make_pair(v1,v2);

  IGraph_it iter_ = graph_.find(e);

  if(iter_ == graph_.end())
    return false;

  e.edge_ = make_pair(v2,v1);

  iter_ = graph_.find(e);

  if(iter_ == graph_.end())
    return false;

  return true;
}

bool ros_graph::removeEdge(Edge_ros e){

  IGraph_it iter_ = graph_.find(e);

  if(iter_ == graph_.end())
    return false;
  else
  {
    graph_.erase(iter_);
    return true;
  }


}

bool ros_graph::removeEdge(Vertex_ros v1, Vertex_ros v2){

  Edge_ros e;
  e.edge_ = make_pair(v1,v2);

  bool val;
  val = removeEdge(e);

  if(val)
    return true;
  else
  {
    e.edge_ = make_pair(v2,v1);
    val = removeEdge(e);

    if(val)
      return true;
    else
      return false;
  }

}

int ros_graph::countVertex(Vertex_ros v1){

  int num_of_edges = 0;
  for(IGraph_it iter_ = graph_.begin(); iter_ != graph_.end(); iter_++)
    if(iter_->edge_.first.index_ == v1.index_ || iter_->edge_.second.index_ == v1.index_)
      num_of_edges++;
  return num_of_edges;
}

struct find_vertex
{
  int id;
  find_vertex(int id) : id(id) {}
  bool operator () ( const Vertex_ros& v ) const
  {
    return v.index_ == id;
  }
};

Eigen::MatrixXf ros_graph::computeAdjacencyMatrix(){

	std::vector<Vertex_ros> vertex_list_visited;
	std::vector<Vertex_ros>::iterator it;
	std::vector<Vertex_ros>::iterator local_it;
	std::vector<std::vector<Vertex_ros> > adjacency_list;

	for(IGraph_it iter = graph_.begin(); iter != graph_.end(); ++iter){

		int index = 0;
		std::vector<Vertex_ros> local_adjacency;
		if(vertex_list_visited.empty()){
			local_adjacency.clear();
			vertex_list_visited.push_back(iter->edge_.first);
			local_adjacency.push_back(iter->edge_.second);
			adjacency_list.push_back(local_adjacency);
		}

		it = std::find_if(vertex_list_visited.begin(), vertex_list_visited.end(), find_vertex(iter->edge_.first.index_));
		if(it == vertex_list_visited.end())
		{
			local_adjacency.clear();
			vertex_list_visited.push_back(iter->edge_.first);
			local_adjacency.push_back(iter->edge_.second);
			adjacency_list.push_back(local_adjacency);
		}
		else{
			local_adjacency.clear();
			index = std::distance(vertex_list_visited.begin(), it);
			local_it = std::find_if(adjacency_list[index].begin(), adjacency_list[index].end(), find_vertex(iter->edge_.second.index_));
			if(local_it == adjacency_list[index].end())
				adjacency_list[index].push_back(iter->edge_.second);

		}

		it = std::find_if(vertex_list_visited.begin(), vertex_list_visited.end(), find_vertex(iter->edge_.second.index_));
		if(it == vertex_list_visited.end())
		{
			local_adjacency.clear();
			vertex_list_visited.push_back(iter->edge_.second);
			local_adjacency.push_back(iter->edge_.first);
			adjacency_list.push_back(local_adjacency);
		}
		else{
			local_adjacency.clear();
			index = std::distance(vertex_list_visited.begin(), it);
			local_it = std::find_if(adjacency_list[index].begin(), adjacency_list[index].end(), find_vertex(iter->edge_.first.index_));
			if(local_it == adjacency_list[index].end())
				adjacency_list[index].push_back(iter->edge_.first);
		}
	}

	//ROS_INFO("Creating adjacency matrix in computation loop");
	Eigen::MatrixXf adjacency_matrix = Eigen::MatrixXf::Zero(adjacency_list.size(),adjacency_list.size());

	for(unsigned int i = 0; i < adjacency_matrix.rows(); i++)
		for(it = adjacency_list[i].begin(); it!=adjacency_list[i].end();++it)
		{
			local_it = std::find_if(vertex_list_visited.begin(), vertex_list_visited.end(), find_vertex(it->index_));
			int adjacency_index = std::distance(vertex_list_visited.begin(),local_it);
			//ROS_INFO_STREAM("Matrix size nxn: "<<adjacency_matrix.rows()<<" row: "<<i<<" col:"<<adjacency_index);
			adjacency_matrix(i,adjacency_index) = 1;
		}

	return adjacency_matrix;

}
Vertex_ros ros_graph::findMaxVertex(){

  std::vector<Vertex_ros> vertex_list_visited;
  std::vector<Vertex_ros>::iterator it;
  int max_vert_count = 0;
  int vert_location = 0;
  Vertex_ros max_vert;

  for(IGraph_it iter = graph_.begin(); iter != graph_.end(); ++iter){

    if(vertex_list_visited.empty()){
      max_vert_count = countVertex(iter->edge_.first);
      vertex_list_visited.push_back(iter->edge_.first);
      vert_location++;
    }


    it = std::find_if(vertex_list_visited.begin(), vertex_list_visited.end(), find_vertex(iter->edge_.first.index_));
    if(it == vertex_list_visited.end())
      if(max_vert_count < countVertex(iter->edge_.first))
      {
        max_vert_count = countVertex(iter->edge_.first);
        vertex_list_visited.push_back(iter->edge_.first);
        vert_location++;
      }

    it = std::find_if(vertex_list_visited.begin(), vertex_list_visited.end(), find_vertex(iter->edge_.second.index_));
    if(it == vertex_list_visited.end())
      if(max_vert_count < countVertex(iter->edge_.second)){
        max_vert_count = countVertex(iter->edge_.second);
        vertex_list_visited.push_back(iter->edge_.second);
        vert_location++;
      }
  }

  for(int i=0;i<vertex_list_visited.size();i++)
    ROS_DEBUG("Vertex index %d",vertex_list_visited[i].index_);

  ROS_DEBUG("Number of Vertices : %d in current graph",number_of_vertices_);
  ROS_DEBUG("Max Vertex Location %d Max Vertex Count %d index %d",vert_location,
            max_vert_count,vertex_list_visited[vert_location - 1].index_);

  if(vert_location > 0)
    return vertex_list_visited[vert_location - 1];
  else
    return vertex_list_visited[0];
}


}
