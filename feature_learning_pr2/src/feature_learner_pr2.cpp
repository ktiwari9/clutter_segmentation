/*********************************************************************
*
*  Copyright (c) 2014, Computational Learning and Motor Control Laboratory
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
 * @b client for pr2 controller manager and feature learning
 *
 */
#include <ros/ros.h>
// Extract features includes
#include <math.h>
#include "feature_learning/extract_features.hpp"
#include "feature_learning/ExtractFeatures.h"

#include <fstream>
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include <action_manager_pr2/ControllerAction.h>
#include <geometry_msgs/PoseStamped.h>

using namespace action_manager_pr2;

enum control_t {HEAD, GRIPPER, ARM};

class action_client_pr2{

public:
	typedef actionlib::SimpleActionClient<ControllerAction> Client;
	action_manager_pr2::ControllerGoal goal_;

private:
	Client ac_;

public:
	action_client_pr2(std::string name):ac_(name,true){
		ROS_INFO("feature_learning_pr2::feature_learner_pr2 Waiting for action server to start.");
		    ac_.waitForServer();
		ROS_INFO("feature_learning_pr2::feature_learner_pr2 Action server started, sending goal.");
	}

	void sendGoal(){

		ROS_INFO("feature_learning_pr2::feature_learner_pr2 Populating Goal Message.");
		ac_.sendGoal(goal_,boost::bind(&action_client_pr2::goalReturned,this,_1,_2),Client::SimpleActiveCallback(),Client::SimpleFeedbackCallback());

	}

	void goalReturned(const actionlib::SimpleClientGoalState& state, const ControllerResultConstPtr& result){

		ROS_INFO("feature_learning_pr2::feature_learner_pr2 Finished in state [%s]", state.toString().c_str());
		ROS_INFO("feature_learning_pr2::feature_learner_pr2 Answer: %d", result->result);
		ros::shutdown();

	}
};

bool callAndRecordFeature(){

	feature_learning::ExtractFeatures extract_feature_srv;

	std::string feature_service("/extract_features_srv");
	// Now get the response from the static segment server and record the adjacency matrix
	//extract_feature_srv.request.call = extract_feature_srv.request.EMPTY;

	bool call_succeeded = false;

	while(!call_succeeded){

		if (!ros::service::call(feature_service, extract_feature_srv))
		{
			ROS_ERROR("feature_learning_pr2::feature_learner_pr2 Call to segmentation service failed");
		}

		if(ros::service::call(feature_service,extract_feature_srv)){
			call_succeeded = true;
			ROS_INFO("feature_learning_pr2::feature_learner_pr2 Service Call succeeded");
		}

		if (extract_feature_srv.response.result == extract_feature_srv.response.FAILURE)
		{
			ROS_ERROR("feature_learning_pr2::feature_learner_pr2 Segmentation service returned error");
			return false;
		}

	}
	ROS_INFO("feature_learning_pr2::feature_learner_pr2: Feature Service succeeded");
	return true;
}


int main(int argc, char **argv){

	ros::init(argc,argv,"test_adjacency_recorder");
	action_client_pr2 ac("Controller");

	if(argc < 1)
	{
		ROS_INFO("feature_learning_pr2::feature_learner_pr2: Usage: execute_action_client <reward_filename>");
		return -1;
	}


	//Now set a bool variable to check till user quits
	bool repeat = true;

	while(repeat)
	{

		ROS_INFO("feature_learning_pr2::feature_learner_pr2: Calling feature service");
		bool success = callAndRecordFeature();

		if(success)
			ROS_INFO("feature_learning_pr2::feature_learner_pr2: Call succeeded ");
		else
			ROS_INFO("feature_learning_pr2::feature_learner_pr2: Call failed ");

		ROS_INFO("feature_learning_pr2::feature_learner_pr2: Calling action server");


		action_manager_msgs::Controller local_goal;
		local_goal.target	= action_manager_msgs::Controller::HEAD;
		local_goal.head.action = action_manager_msgs::Head::LOOK;
		local_goal.head.frame_id = "/base_link";
		geometry_msgs::Point target_point;
		target_point.x = 5.0; target_point.y = 1.0; target_point.z = 1.2;
		local_goal.head.pose.position = target_point;
		local_goal.header.stamp = ros::Time::now();

		ac.goal_.controller = local_goal;
		ROS_INFO("Sending goal target:%d",ac.goal_.controller.target);
		ac.sendGoal();

		char answer;
		std::cout<<"Call feature extraction again (y/n) :"<<std::endl;
		std::cin>>answer;

		if(std::cin.fail() || answer == 'n')
			repeat = false;

	}
}




