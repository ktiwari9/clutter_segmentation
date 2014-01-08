/*********************************************************************
*
*  Copyright (c) 2013, Computational Learning and Motor Control Laboratory
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
 * @b client for calling the execute action action server
 *
 */
#include <fstream>
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
// Static Segment includes
#include "static_segmentation/StaticSegment.h"
#include "static_segmentation/StaticSeg.h"
#include "static_segmentation/static_segmenter.hpp"
//Graph Module includes
#include "graph_module/EGraph.h"
#include <graph_module/graph_module.hpp>

// beta distributions include from boost
#include <boost/math/special_functions/digamma.hpp>
#define BOOST_HAS_TR1_TUPLE 1
#include <boost/tr1/detail/config.hpp>
#undef BOOST_HAS_INCLUDE_NEXT
#include <boost/tr1/tuple.hpp>
#include <boost/math/special_functions/beta.hpp>


Eigen::MatrixXf getAdjacencyFromGraph(std::vector<static_segmentation::StaticSeg>& graph_msg){

	std::vector<graph::ros_graph> graph_list;

	int total_vertices = 0;
//	ROS_INFO("Converting graph message to graph");
	for(unsigned int i = 0; i < graph_msg.size(); i++){

		graph::ros_graph single_graph;
		single_graph.buildGraph(graph_msg[i].graph);

		if(single_graph.number_of_vertices_ <= 1)
			continue;

		total_vertices += single_graph.number_of_vertices_;

		//ROS_INFO("Number of graph vertices %d",single_graph.number_of_vertices_);

		graph_list.push_back(single_graph);
	}

	// Now get adjaceny matrix from graph
	Eigen::MatrixXf adjacency = Eigen::MatrixXf::Zero(total_vertices,total_vertices);
	long int row_index = 0, col_index = 0;

	//ROS_INFO_STREAM("Total number of vertices "<<total_vertices<<std::endl<<" Graph List Size "<<graph_list.size());

	for(unsigned int i = 0; i < graph_list.size(); i++){

		Eigen::MatrixXf local_adjacency = graph_list[i].getAdjacencyMatrix();
		//ROS_INFO_STREAM("Current adjacency size nxn:"<<local_adjacency.rows()<<std::endl<<local_adjacency);

		adjacency.block(row_index,col_index,local_adjacency.rows(),local_adjacency.cols()) = local_adjacency;
		row_index += local_adjacency.rows(); col_index += local_adjacency.cols();

	}
	return adjacency;
}

bool callAndRecordAdjacency(Eigen::MatrixXf &adjacency){

	adjacency.resize(0,0);
	static_segmentation::StaticSegment staticsegment_srv;

	std::string static_service("/static_segment_srv");
	// Now get the response from the static segment server and record the adjacency matrix
	staticsegment_srv.request.call = staticsegment_srv.request.EMPTY;
	bool call_succeeded = false;

	while(!call_succeeded){

                if (!ros::service::call(static_service, staticsegment_srv))
                   {
//                   ROS_ERROR("Call to segmentation service failed");
                     exit(0);
                   }

		if(ros::service::call(static_service,staticsegment_srv)){
			call_succeeded = true;
//			ROS_INFO("Service Call succeeded");
			std::vector<static_segmentation::StaticSeg> graph_msg = staticsegment_srv.response.graph_queue;
			adjacency = getAdjacencyFromGraph(graph_msg);
		}

               if (staticsegment_srv.response.result == staticsegment_srv.response.FAILURE)
                {
//               ROS_ERROR("Segmentation service returned error");
                 exit(0);
                }	

	}
	return true;
}

float compute_entropy(float a, float b){

	float entropy_value = log(boost::math::beta(a,b)) - (a - 1)*boost::math::digamma(a) - (b-1)*boost::math::digamma(b) + (a+b-2)*boost::math::digamma(a+b);
	return entropy_value; // clean up later
}

int main(int argc, char **argv){

	ros::init(argc,argv,"test_adjacency_recorder");

	if(argc < 2)
	{
		ROS_INFO("execute_action_client: Usage: execute_action_client <reward_filename>");
		return -1;
	}


	std::string filename(argv[1]);

	//Now set a bool variable to check till user quits
	bool repeat = true;
	int iteration_number = 1;

	float alpha_val = 1.0, beta_val = 1.0;
	int count_pos = 0, count_neg = 0;


	// priors alpha = 0.5 and beta = 0.5
	// Normalized update, alpha_1 = alpha_0 + N_1/N; beta_1 = beta_0 + (1 - N_1/N);
	// Unnormalized update alpha_1 = alpha_0 + N_1; beta_1 = beta_0 + (N - N_1);

        // Record matrix once before and once after the action
        Eigen::MatrixXf adjacency;
        bool success_before = callAndRecordAdjacency(adjacency);
        float adj_sum_old = adjacency.sum();

	while(repeat)
	{

		// CURRENT IMPLEMENTATION OF ONLY ONE WHOLE BREAKING EVENT FOR THE ENTIRE ADJACENCY MATRIX
		// NORMALIZED UPDATE
		// TODO: THINK ABOUT EXTENDING TO INDIVIDUAL EDGES

		// Storing the adjacency matrix
		std::string eigen_before_filename(filename+"_beforeADJ_"+ boost::lexical_cast<std::string>(iteration_number)+".txt");
		ofstream ofs_before(eigen_before_filename.c_str(),ios::out | ios::trunc);
		if(ofs_before)
		{
			// instructions
			ofs_before << adjacency;
			ofs_before.close();
		}
		else  // sinon
		{
			std::cerr << "Erreur à l'ouverture !" << std::endl;
		}


		int action = 0;
//		ROS_INFO_STREAM("Perform an action" << std::endl <<"Manipulate clutter and then press enter"<<std::endl);
		std::cout<<" Enter a number if action completed ";
		std::cin>>action;


		// Recording the after adjacency matrix
		std::cout<<"Adjacency before "<<adj_sum_old<<std::endl;
		bool success_after = callAndRecordAdjacency(adjacency);
		std::cout<<"Adjacency after "<<adjacency.sum()<<std::endl;
		float prev_entropy = compute_entropy(alpha_val,beta_val);
		std::cout<<"Before: alpha value "<<alpha_val<<" Beta Value "<<beta_val<<std::endl;
		if(adj_sum_old > adjacency.sum())
		{
			count_pos += 1;
			//alpha_val += ((adj_sum_old - adjacency.sum())/2)*static_cast<float>(count_pos)/static_cast<float>(iteration_number);
			alpha_val += static_cast<float>(count_pos)/static_cast<float>(iteration_number);
                        std::cout<<"Updating positive count "<<std::endl;
		}

		if(adj_sum_old < adjacency.sum())
		{
			count_neg += 1;
			//beta_val += ((adjacency.sum() - adj_sum_old)/2)*static_cast<float>(count_neg)/static_cast<float>(iteration_number);
			beta_val += static_cast<float>(count_neg)/static_cast<float>(iteration_number);
                        std::cout<<"Updating negative count "<<std::endl;
		}
		float new_entropy = compute_entropy(alpha_val,beta_val);
		std::cout<<"After: alpha value "<<alpha_val<<" Beta Value "<<beta_val<<std::endl;
                std::cout<<"Pos :"<<count_pos<<" Neg :"<<count_neg<<" iteration_number :"<<iteration_number<<std::endl;

		// Storing the adjacency matrix
		std::string eigen_after_filename(filename+"_afterADJ_"+ boost::lexical_cast<std::string>(iteration_number)+".txt");
		ofstream ofs_after(eigen_after_filename.c_str(),ios::out | ios::trunc);

		if(ofs_after)
		{
			// instructions
			ofs_after << adjacency;
			ofs_after.close();
		}
		else  // sinon
		{
			std::cerr << "Erreur à l'ouverture !" << std::endl;
		}

		char answer;

		// to measure +ve reward, because beta entropy -> max = 0 ; min = -inf
		std::cout<<"Change in Entropy (-new + prev) = " << -new_entropy<<" + "<<prev_entropy<<" = "<<-new_entropy+prev_entropy <<std::endl;

		std::cout<<"Do you want to continue (y/n) ";
		std::cin>>answer;

		iteration_number++;
		if(std::cin.fail() || answer == 'n')
			repeat = false;
                
                adj_sum_old = adjacency.sum();

	}

}
