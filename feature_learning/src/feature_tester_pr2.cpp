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

// Extract features includes
#include <math.h>
#include "feature_learning/extract_features.hpp"
#include "feature_learning/ExtractFeatures.h"

#include <fstream>
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
// Static Segment includes
#include "static_segmentation/StaticSegment.h"
#include "static_segmentation/StaticSeg.h"
#include "static_segmentation/static_segmenter.hpp"
//Graph Module includes



bool callAndRecordFeature(){

	feature_learning::ExtractFeatures extract_feature_srv;

	std::string feature_service("/extract_features_srv");
	// Now get the response from the static segment server and record the adjacency matrix
	//extract_feature_srv.request.call = extract_feature_srv.request.EMPTY;
	bool call_succeeded = false;

	while(!call_succeeded){

		if (!ros::service::call(feature_service, extract_feature_srv))
		{
			ROS_ERROR("Call to segmentation service failed");
		}

		if(ros::service::call(feature_service,extract_feature_srv)){
			call_succeeded = true;
			ROS_INFO("Service Call succeeded");
		}

		if (extract_feature_srv.response.result == extract_feature_srv.response.FAILURE)
		{
			ROS_ERROR("Segmentation service returned error");
			return false;
		}

	}
	return true;
}


int main(int argc, char **argv){

	ros::init(argc,argv,"test_adjacency_recorder");

	if(argc < 1)
	{
		ROS_INFO("execute_action_client: Usage: execute_action_client <reward_filename>");
		return -1;
	}


	//Now set a bool variable to check till user quits
	bool repeat = true;
        
	while(repeat)
	{

		bool success = callAndRecordFeature();

		if(success)
			ROS_INFO("Call succeeded ");
		else
			ROS_INFO("Call failed ");

		char answer;
		std::cout<<"Call feature extraction again (y/n) :"<<std::endl;
		std::cin>>answer;

		if(std::cin.fail() || answer == 'n')
			repeat = false;                
                
	}
}
