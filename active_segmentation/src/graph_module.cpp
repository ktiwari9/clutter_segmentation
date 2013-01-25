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

#include "active_segmentation/graph_module.hpp"

namespace active_segmentation {

graph_module::graph_module(int number_of_vertices):
	graph_(){

	number_of_vertices_ = number_of_vertices;
}

void graph_module::addEdge(Vertex v1, Vertex v2, double weight){

	Edge e;
	e.edge_ = make_pair(v1,v2);
	e.weight_ = weight;

	graph_.insert(e);
}

void graph_module::addEdge(int node_1, double centroid_1, int node_2, double centroid_2, double weight){

	Vertex v1, v2;
	v1.index_ = node_1;
	v1.centroid_ = centroid_1;
	v2.index_ = node_2;
	v2.centroid_ = centroid_2;

	Edge e;
	e.edge_ = make_pair(v1,v2);
	e.weight_ = weight;

	graph_.insert(e);
}

bool graph_module::findVertex(Vertex v){

	for(iter_ = graph_.begin(); iter_ != graph_.end(); iter_++)
		if(iter_->edge_.first.index_ == v.index_ || iter_->edge_.second.index_ == v.index_ )
			return true;

	return false;

}

bool graph_module::findVertex(int index){

	for(iter_ = graph_.begin(); iter_ != graph_.end(); iter_++)
		if(iter_->edge_.first.index_ == index || iter_->edge_.second.index_ == index)
			return true;

	return false;

}

bool graph_module::findEdge(Edge e){

	iter_ = graph_.find(e);

	if(iter_ == graph_.end())
		return false;
	else
		return true;
}

bool graph_module::findEdge(Vertex v1, Vertex v2){

	Edge e;
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

bool graph_module::removeEdge(Edge e){

	iter_ = graph_.find(e);

	if(iter_ == graph_.end())
		return false;
	else
	{
		graph_.erase(iter_);
		return true;
	}


}

bool graph_module::removeEdge(Vertex v1, Vertex v2){

	Edge e;
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

int graph_module::countVertex(Vertex v1){

	// Find a better way to do this
	return 1;
}

std::vector<Vertex> graph_module::findMaxVertex(){

	std::vector<Vertex> vertex_list;
	int max_vert = 0;
	for(iter_ = graph_.begin(); iter_ != graph_.end(); iter_++){

		if(max_vert < countVertex(iter_->edge_.first))
			vertex_list.push_back(iter_->edge_.first);

		if(max_vert < countVertex(iter_->edge_.second))
			vertex_list.push_back(iter_->edge_.second);
	}

	return vertex_list;
}


}
