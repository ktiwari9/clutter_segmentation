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
#include <opencv2/opencv.hpp>
#include <vector>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "static_segmentation/StaticSegment.h"
#include <geometry_msgs/Polygon.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <tr1/unordered_set> // should I use an un ordered set?
#include <tr1/functional>

// boost graph library includes
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/undirected_graph.hpp>

using namespace std;

namespace active_segmentation {

// Defining Vertex Structure
struct Vertex{

	int index_;
	double x_,y_;
};

// Defining Edge as a pait of vertices with a weight
struct Edge{

	std::pair<Vertex,Vertex> edge_;
	double weight_;
};

// Defining HashFunction as a product of both index vertices
struct HashFunction {
  size_t operator()(const Edge& x) const { return std::tr1::hash<int>()(x.edge_.first.index_*x.edge_.second.index_); }
};


struct SetEqual {
    bool operator() (const Edge& a, const Edge& b) const {

    	if((a.edge_.first.index_ == b.edge_.first.index_ && a.edge_.second.index_ == b.edge_.second.index_) ||
    			(a.edge_.first.index_ == b.edge_.second.index_ && a.edge_.second.index_ == b.edge_.first.index_))
    		return true;
    	else
    		return false;}
};



class graph_module{

friend class active_segment;

public:

	typedef std::tr1::unordered_set<Edge,HashFunction,SetEqual> Graph;
	typedef std::tr1::unordered_set<Edge,HashFunction,SetEqual>::iterator Graph_it;

	// setS = std::set - container for edges (can be added and removed in any order) Statisfy Sequence or Associative
	// set enforces absence of multigraph
	// vecS = std::vector container for vectors ; Satisfy sequence or random access
	//typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS,Vertex, Edge> Graph;

protected:

	//defining boost Graph
	int number_of_vertices_;

	Graph graph_;

	Graph_it iter_;

public:

	graph_module(int number_of_vertices);

	graph_module();

	~graph_module();

	bool findVertex(int index);

	bool findVertex(Vertex v);

	void addEdge(Edge new_edge){ graph_.insert(new_edge);}

	void addEdge(Vertex v1, Vertex v2, double weight);

	void addEdge(int node_1, double centroid_x1, double centroid_y1,
			int node_2, double centroid_x2, double centroid_y2, double weight);

	bool findEdge(Edge e);

	bool findEdge(Vertex v1, Vertex v2);

	bool removeEdge(Edge e);

	bool removeEdge(Vertex v1, Vertex v2);

	int countVertex(Vertex v1);

	Vertex findMaxVertex();

	void displayGraph(); // Need to think about how to do this!

};

}

#endif
