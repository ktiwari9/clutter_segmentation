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

#ifndef GRAPH_MODULE_HPP
#define GRAPH_MODULE_HPP

#include <ros/ros.h>
#include <vector>
#include <tr1/unordered_set>
#include <tr1/functional>
#include <graph_module/EGraph.h> // Defines Edge based Graph
#include <graph_module/VGraph.h> // Defines Vertex based Graph

// boost graph library includes
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/undirected_graph.hpp>

using namespace std;

namespace graph{

// Defining Vertex Structure
struct Vertex_ros{

	int index_;
	double x_,y_;
};

// Defining Edge as a pait of vertices with a weight
struct Edge_ros{

	std::pair<Vertex_ros,Vertex_ros> edge_;
	double weight_;
};

// Defining HashFunction as a product of both index vertices
struct HashFunction {
  size_t operator()(const Edge_ros& x) const { return std::tr1::hash<int>()(x.edge_.first.index_*x.edge_.second.index_); }
};


struct SetEqual {
    bool operator() (const Edge_ros& a, const Edge_ros& b) const {

    	if((a.edge_.first.index_ == b.edge_.first.index_ && a.edge_.second.index_ == b.edge_.second.index_) ||
    			(a.edge_.first.index_ == b.edge_.second.index_ && a.edge_.second.index_ == b.edge_.first.index_))
    		return true;
    	else
    		return false;}
};



class ros_graph{

public:

	typedef std::tr1::unordered_set<Edge_ros,HashFunction,SetEqual> IGraph;
	typedef std::tr1::unordered_set<Edge_ros,HashFunction,SetEqual>::iterator IGraph_it;

public:

	//defining boost Graph
	int number_of_vertices_;

	IGraph graph_;

	IGraph_it iter_;

	ros_graph(int number_of_vertices);

	ros_graph();

	bool buildGraph(graph_module::EGraph in_graph_msg); // building an edge based graph

	bool buildGraph(graph_module::VGraph in_graph_msg); // build a vertex based graph

	~ros_graph();

	bool findVertex(int index);

	bool findVertex(Vertex_ros v);

	void addEdge(Edge_ros new_edge){ graph_.insert(new_edge);}

	void addEdge(Vertex_ros v1, Vertex_ros v2, double weight);

	void addEdge(int node_1, double centroid_x1, double centroid_y1,
			int node_2, double centroid_x2, double centroid_y2, double weight);

	bool findEdge(Edge_ros e);

	bool findEdge(Vertex_ros v1, Vertex_ros v2);

	bool removeEdge(Edge_ros e);

	bool removeEdge(Vertex_ros v1, Vertex_ros v2);

	int countVertex(Vertex_ros v1);

	Vertex_ros findMaxVertex();

	graph_module::EGraph convertToEGraphMsg();

	graph_module::VGraph convertToVGraphMsg(); // TODO: Not defined yet

};

}

#endif
