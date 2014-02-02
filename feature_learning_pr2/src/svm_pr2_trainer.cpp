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
#include <visualization_msgs/Marker.h>

using namespace action_manager_pr2;

enum control_t {HEAD, GRIPPER, ARM};

class action_client_pr2{

public:
	typedef actionlib::SimpleActionClient<ControllerAction> Client;
	action_manager_pr2::ControllerGoal goal_;
	std::vector<geometry_msgs::PointStamped> action_point_;
	std::vector<int> action_indices_;
	bool action_result_;

private:
	Client ac_;

public:
	action_client_pr2(std::string name):ac_(name,true),action_result_(false){
		ROS_INFO("feature_learning_pr2::svm_pr2_trainer Waiting for action server to start.");
		ac_.waitForServer();
		ROS_INFO("feature_learning_pr2::svm_pr2_trainer Action server started, sending goal.");
	}

	void sendGoal(){

		ROS_INFO("feature_learning_pr2::svm_pr2_trainer Populating Goal Message.");
		ac_.sendGoal(goal_,boost::bind(&action_client_pr2::goalReturned,this,_1,_2),Client::SimpleActiveCallback(),Client::SimpleFeedbackCallback());
		ac_.waitForResult();

	}

	void goalReturned(const actionlib::SimpleClientGoalState& state, const ControllerResultConstPtr& result){

		ROS_INFO("feature_learning_pr2::svm_pr2_trainer Finished in state [%s]", state.toString().c_str());
		ROS_INFO("feature_learning_pr2::svm_pr2_trainer Action Manager Return value: %d", result->result);
		action_result_ = result->result;
		//ros::shutdown();

	}

	bool callAndRecordFeature(feature_learning::ExtractFeatures& extract_feature_srv){

		std::string feature_service("/extract_features_srv");

		ros::service::waitForService(feature_service);

		if(ros::service::call(feature_service,extract_feature_srv)){
			ROS_INFO("feature_learning_pr2::svm_pr2_trainer Service Call succeeded");
			action_point_ = extract_feature_srv.response.training_centers;
			action_indices_ = extract_feature_srv.response.indicies;
		}

		if (extract_feature_srv.response.result == extract_feature_srv.response.FAILURE)
		{
			ROS_ERROR("feature_learning_pr2::svm_pr2_trainer Segmentation service returned error");
			return false;
		}

		return true;
	}

	void actuateHead(){

		goal_.controller.target = action_manager_msgs::Controller::HEAD;
		goal_.controller.header.stamp = ros::Time::now();
		goal_.controller.head.frame_id = "/base_link";

		float x,y,z;
		ROS_INFO("feature_learning_pr2::svm_pr2_trainer:  Enter position (5.0 1.0 1.2) in base_link: \n x y z:");
		std::cin >> x >> y >> z;

		goal_.controller.head.pose.position.x = x;
		goal_.controller.head.pose.position.y = y;
		goal_.controller.head.pose.position.z = z;

		int target;
		ROS_INFO("feature_learning_pr2::svm_pr2_trainer: Track target (1) or look at target (2)? \n Enter Choice (1-2)");
		std::cin >> target;

		if(target)
			goal_.controller.head.action = action_manager_msgs::Head::TRACK;
		else
			goal_.controller.head.action = action_manager_msgs::Head::LOOK;
	}

	void actuateGripper(){

		goal_.controller.target = action_manager_msgs::Controller::GRIPPER;

		int hand;
		ROS_INFO("feature_learning_pr2::svm_pr2_trainer:  Select Hand (1-right, 0-left):");
		std::cin >> hand;

		if(hand)
			goal_.controller.gripper.gripper = goal_.controller.gripper.GRIPPER_R;
		else
			goal_.controller.gripper.gripper = goal_.controller.gripper.GRIPPER_L;

		int action;
		ROS_INFO("feature_learning_pr2::svm_pr2_trainer:  Select Action (1-close, 0-open):");
		std::cin >> action;

		if(action)
			goal_.controller.gripper.action = goal_.controller.gripper.CLOSE;
		else
			goal_.controller.gripper.action = goal_.controller.gripper.OPEN;

		goal_.controller.header.stamp = ros::Time::now();
	}

	void actuateArm(){

		goal_.controller.target = action_manager_msgs::Controller::ARM;
		int action;
		ROS_INFO("feature_learning_pr2::svm_pr2_trainer:  Select Action (0-TUCK, 1-STRETCH,2-HOME,3-GOZERO,4-PREGRASP,5-GRASP,6-PUSH):");
		std::cin >> action;
		goal_.controller.arm.action = action;
		//goal_.controller.arm.frame_id = "/base_link";
		if(action < 5)
		{
			int arm;
			ROS_INFO("Select Arm (0-LEFT,1-RIGHT)");
			std::cin>>arm;
			goal_.controller.arm.arm = arm;
			return;
		}
		else
		{
			if(action == 5)
			{
				//float x,y,z;
				//ROS_INFO("feature_learning_pr2::svm_pr2_trainer:  Enter place in base frame(1.0 1.0 1.0): \n x y z:");
				//std::cin >> x >> y >> z;
				goal_.controller.arm.end_pose.position.x = 1.0;
				goal_.controller.arm.end_pose.position.y = -1.0;
				goal_.controller.arm.end_pose.position.z = 1.0;
			}
			else
			{
				int direction;
				ROS_INFO_STREAM("feature_learning_pr2::svm_pr2_trainer:  Push direction choices :\n"<<"0-FRONT\n"<<"3-FROM_RIGHT_SIDEWAYS\n"<<
						"4-FROM_RIGHT_UPRIGHT\n"<<"5-FROM_LEFT_UPRIGHT\n"<<"6-FROM_RIGHT_SIDEWAYS\n"<<"Enter your choice: ");
				std::cin >> direction;
				goal_.controller.arm.direction = direction;

			}
		}

	}

};

visualization_msgs::Marker getMarker(int i){

	visualization_msgs::Marker local_marker;

	local_marker.header.stamp = ros::Time();
	local_marker.ns = "extract_features";
	local_marker.id = i;
	local_marker.type = visualization_msgs::Marker::CUBE;
	local_marker.action = visualization_msgs::Marker::ADD;
	local_marker.scale.x = 0.1;
	local_marker.scale.y = 0.1;
	local_marker.scale.z = 0.1;
	local_marker.color.a = 1.0;
	local_marker.color.r = 0.0;
	local_marker.color.g = 1.0;
	local_marker.color.b = 1.0;
	local_marker.pose.orientation.x = 0.0;
	local_marker.pose.orientation.y = 0.0;
	local_marker.pose.orientation.z = 0.0;
	local_marker.pose.orientation.w = 1.0;

	return local_marker;

}

int main(int argc, char **argv){

	ros::init(argc,argv,"svm_pr2_trainer");
	ros::NodeHandle nh;
	action_client_pr2 ac("/pr2_action_interface");
	ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("/manipulation_marker", 1);
	ros::Publisher pub_place = nh.advertise<visualization_msgs::Marker>("/place_location_marker", 1);
	//Now set a bool variable to check till user quits
	if(argc < 2)
	{
		ROS_INFO("svm_pr2_trainer: Usage: svm_pr2_trainer <reward_filename>");
		return -1;
	}


	std::string base_filename(argv[1]);
	ROS_INFO_STREAM("feature_learning_pr2::svm_pr2_trainer: Base file name %s"<<base_filename);
	bool repeat = true;

	int counter = 0;

	while(repeat)
	{
		std::stringstream target_filename;
		int choice = 1;
		ROS_INFO("feature_learning_pr2::svm_pr2_trainer: What Body Part would you like to control (1-3)? \n 1) Head \n 2) Gripper \n 3) Arm \n Enter Choice (1-3):");
		std::cin >> choice;

		feature_learning::ExtractFeatures extract_feature_srv;
		bool success=false;
		switch(choice){

		case(1) :
		ac.actuateHead();
		ac.sendGoal(); // Technically no difference between tracking and looking
		break;

		case(2) :
		ac.actuateGripper();
		ac.sendGoal();
		break;

		case(3) :
		ac.actuateArm();
		extract_feature_srv.request.action = extract_feature_srv.request.TRAIN;
		target_filename << base_filename<<"_"<< ac.goal_.controller.arm.action<<"_"<<boost::lexical_cast<std::string>(counter);
		extract_feature_srv.request.filename = target_filename.str();

		ROS_INFO("feature_learning_pr2::svm_pr2_trainer: Now calling extract feature service with filename %s",target_filename.str().c_str());
		if(ac.goal_.controller.arm.action > 4)
		{
			success = ac.callAndRecordFeature(extract_feature_srv);
			if(success)
			{
				// now loop through all return messages
				for(size_t index = 0; index < ac.action_point_.size(); index++)
				{
					ac.goal_.controller.arm.start_pose.position = ac.action_point_[index].point;
					ac.goal_.controller.arm.frame_id = ac.action_point_[index].header.frame_id;
					ac.goal_.controller.header.frame_id = ac.action_point_[index].header.frame_id;
					ac.goal_.controller.header.stamp = ac.action_point_[index].header.stamp;

					visualization_msgs::Marker location_marker = getMarker(index);
					location_marker.header = ac.action_point_[index].header;
					location_marker.pose.position =  ac.action_point_[index].point;
					pub.publish(location_marker);

					if(ac.goal_.controller.arm.action == 5)
					{
						visualization_msgs::Marker place_location_marker = getMarker(index);
						place_location_marker.color.r = 1.0;
						place_location_marker.color.g = 1.0;
						place_location_marker.color.b = 0.0;
						place_location_marker.header = ac.action_point_[index].header;
						place_location_marker.pose.position =  ac.goal_.controller.arm.end_pose.position;
						pub_place.publish(place_location_marker);

					}

					ac.sendGoal();

					if(ac.action_result_) // If action succeeds
					{
						std::string reward_filename(target_filename.str()+"_reward_"+boost::lexical_cast<std::string>(ac.action_indices_[index])+".txt");
						ofstream ofs(target_filename.str().c_str(),ios::out | ios::trunc);
						int label;
						ROS_INFO("feature_learning_pr2::svm_pr2_trainer: Action succeeded , Enter label: (1,-1)");
						std::cin >> label;
						if(ofs)
						{
							// instructions
							ofs << label;
							ofs.close();
						}
						else
						{
							ROS_ERROR("feature_learning_pr2::svm_pr2_trainer: Could not open output file");
						}
						break;
					}
					else // If action fails
					{
						std::string reward_filename(target_filename.str()+"_reward_"+boost::lexical_cast<std::string>(ac.action_indices_[index])+".txt");
						ofstream ofs(target_filename.str().c_str(),ios::out | ios::trunc);
						if(ofs)
						{
							// instructions
							ofs << -1;
							ofs.close();
						}
						else
						{
							ROS_ERROR("feature_learning_pr2::svm_pr2_trainer: Could not open output file");
						}

					}
				}
			}
			else
				ROS_INFO("feature_learning_pr2::svm_pr2_trainer: Feature Extraction failed");
		}
		else
			ac.sendGoal();

		break;

		default:
			ROS_INFO("feature_learning_pr2::svm_pr2_trainer: Incorrect selection");
			ac.action_result_ = false;
			break;
		}

		char answer;
		ROS_INFO("Call feature extraction again (y/n) :");
		std::cin>>answer;

		if(std::cin.fail() || answer == 'n')
			repeat = false;
	}
}

