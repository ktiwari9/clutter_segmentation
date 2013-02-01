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

	if(in_graph_msg.edges.size() == 0)
		return false;
	else{
		graph_.clear();
		for(int i=0; i<in_graph_msg.edges.size(); i++){

			addEdge(in_graph_msg.edges[i].start.index,in_graph_msg.edges[i].start.point.x,in_graph_msg.edges[i].start.point.y,
					in_graph_msg.edges[i].end.index,in_graph_msg.edges[i].end.point.x,in_graph_msg.edges[i].end.point.y,
					in_graph_msg.edges[i].weight);
		}

		return true;
	}

}

bool ros_graph::buildGraph(graph_module::VGraph in_graph_msg){

	return false;
}

graph_module::EGraph ros_graph::convertToEGraphMsg(){

	graph_module::EGraph out_msg;
	for(iter_=graph_.begin(); iter_!=graph_.end(); iter_++){

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

	for(iter_ = graph_.begin(); iter_ != graph_.end(); iter_++)
		if(iter_->edge_.first.index_ == v.index_ || iter_->edge_.second.index_ == v.index_ )
			return true;

	return false;

}

bool ros_graph::findVertex(int index){

	for(iter_ = graph_.begin(); iter_ != graph_.end(); iter_++)
		if(iter_->edge_.first.index_ == index || iter_->edge_.second.index_ == index)
			return true;

	return false;

}

bool ros_graph::findEdge(Edge_ros e){

	iter_ = graph_.find(e);

	if(iter_ == graph_.end())
		return false;
	else
		return true;
}

bool ros_graph::findEdge(Vertex_ros v1, Vertex_ros v2){

	Edge_ros e;
	e.edge_ = make_pair(v1,v2);

	iter_ = graph_.find(e);

	if(iter_ == graph_.end())
		return false;

	e.edge_ = make_pair(v2,v1);

	iter_ = graph_.find(e);

	if(iter_ == graph_.end())
		return false;

	return true;
}

bool ros_graph::removeEdge(Edge_ros e){

	iter_ = graph_.find(e);

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
	for(iter_ = graph_.begin(); iter_ != graph_.end(); iter_++)
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

Vertex_ros ros_graph::findMaxVertex(){

	std::vector<Vertex_ros> vertex_list_visited;
	std::vector<Vertex_ros>::iterator it;
	int max_vert_count = 0;
	int vert_location = 0;
	Vertex_ros max_vert;

	for(iter_ = graph_.begin(); iter_ != graph_.end(); iter_++){

		it = std::find_if(vertex_list_visited.begin(), vertex_list_visited.end(), find_vertex(iter_->edge_.first.index_));
		if(it != vertex_list_visited.end())
			if(max_vert_count < countVertex(iter_->edge_.first))
			{
				max_vert_count = countVertex(iter_->edge_.first);
				vertex_list_visited.push_back(iter_->edge_.first);
				vert_location++;
			}

		it = std::find_if(vertex_list_visited.begin(), vertex_list_visited.end(), find_vertex(iter_->edge_.second.index_));
		if(it != vertex_list_visited.end())
			if(max_vert_count < countVertex(iter_->edge_.second)){

				max_vert_count = countVertex(iter_->edge_.second);
				vertex_list_visited.push_back(iter_->edge_.second);
				vert_location++;
			}

	}

	return vertex_list_visited[vert_location-1];
}


}
