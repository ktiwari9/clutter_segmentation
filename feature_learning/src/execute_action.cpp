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
 * \author Bharath Sankaran & Alexander Herzhog
 *
 * @b source file for the executing various actions while testing the feature selection pipeline
 */

#include "feature_learning/execute_action.hpp"
// To get gripper frame
#include <grasp_template_planning/grasp_planning_params.h>

namespace feature_learning {

execute_action::execute_action(ros::NodeHandle & nh):
		nh_(nh),nh_priv_("~"),
		action_server_(nh_,"execute_action",boost::bind(&execute_action::executeCallBack,this,_1),false),
		poke_object_(),world_state_(),
		manipulation_object_l_(2),manipulation_object_r_(1){

	// Start the action server
	action_server_.start();
	tabletop_client_ = nh_priv_.serviceClient<tabletop_segmenter::TabletopSegmentation>("/tabletop_segmentation");
	extract_feature_client_ = nh_priv_.serviceClient<ExtractFeatures>("/extract_features_srv")
}

execute_action::~execute_action(){}

bool execute_action::getTransform(tf::StampedTransform& transform, const string& from, const string& to) const
{
	tf::TransformListener listener;
	bool result = false;
	try
	{
		if (!listener.waitForTransform(from, to, ros::Time(0), ros::Duration(3.0)))
		{
			ROS_DEBUG_STREAM("feature_learning::execute_action: Wait for transform timed out! "
					"Tried to transform from " << from << " to " << to);
		}
		else
		{
			listener.lookupTransform(from, to, ros::Time(0), transform);
			result = true;
		}
	}
	catch (tf::TransformException ex)
	{
		ROS_DEBUG("feature_learning::execute_action: %s", ex.what());
	}

	return result;
}

geometry_msgs::PoseStamped execute_action::getSurfacePose(){

	/* obtain object cluster and table pose */
	sensor_msgs::PointCloud2 cluster;
	grasp_template::GraspTemplateParams params;
	geometry_msgs::PoseStamped table_pose;
	{
		tabletop_segmenter::TabletopSegmentation tabletop_service;
		tf::TransformListener listener;

		ROS_INFO("feature_learning::execute_action: connecting to tabletop_segmenter ...");
		while (!tabletop_client_.waitForExistence(ros::Duration(1.0)) && ros::ok())
		{
			ros::Rate r(1);
			r.sleep();
		}
		if (!tabletop_client_.call(tabletop_service))
		{
			ROS_INFO("feature_learning::execute_action: Could not get object cluster from tabletop_segmenter.");
			return false;
		}

		tf::StampedTransform base_to_tableworld;

		getTransform(base_to_tableworld,tabletop_service.response.table.pose.header.frame_id,params.frameBase());
		tf::Transform table_to_tableworld;
		tf::poseMsgToTF(tabletop_service.response.table.pose.pose, table_to_tableworld);
		tf::Transform table_to_base = base_to_tableworld.inverse()*table_to_tableworld;

		table_pose.header.stamp = ros::Time::now();
		table_pose.header.frame_id = params.frameBase();
		table_pose.pose.position.x = table_to_base.getOrigin().x();
		table_pose.pose.position.y = table_to_base.getOrigin().y();
		table_pose.pose.position.z = table_to_base.getOrigin().z();
		table_pose.pose.orientation.w = table_to_base.getRotation().w();
		table_pose.pose.orientation.x = table_to_base.getRotation().x();
		table_pose.pose.orientation.y = table_to_base.getRotation().y();
		table_pose.pose.orientation.z = table_to_base.getRotation().z();
	}

	return table_pose;

}

geometry_msgs::PoseStamped execute_action::getGripperPose(){

	/* record gripper pose */
	geometry_msgs::PoseStamped gripper_pose;
	{
		tf::TransformListener listener;
		tf::StampedTransform transform;
		grasp_template::GraspTemplateParams params;
		grasp_template_planning::GraspPlanningParams plan_params;
		try
		{
			if (!listener.waitForTransform(params.frameBase(), plan_params.frameGripper(),
					ros::Time(0), ros::Duration(2)))
			{
				ROS_ERROR_STREAM("Waiting for transform, from " << params.frameBase()
						<< " to " << plan_params.frameGripper() << " timed out.");
				return -1;
			}
			listener.lookupTransform(params.frameBase(), plan_params.frameGripper(),
					ros::Time(0), transform);
			gripper_pose.header.frame_id = params.frameBase();
			gripper_pose.header.stamp = ros::Time::now();
			gripper_pose.pose.position.x = transform.getOrigin().x();
			gripper_pose.pose.position.y = transform.getOrigin().y();
			gripper_pose.pose.position.z = transform.getOrigin().z();
			gripper_pose.pose.orientation.x = transform.getRotation().x();
			gripper_pose.pose.orientation.y = transform.getRotation().y();
			gripper_pose.pose.orientation.z = transform.getRotation().z();
			gripper_pose.pose.orientation.w = transform.getRotation().w();
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
		}
	}

	return gripper_pose;
}

geometry_msgs::PoseStamped execute_action::getViewpointPose(){

	/* obtain viewpoint */
	geometry_msgs::PoseStamped viewpoint_pose;
	{
		tf::TransformListener listener;
		tf::StampedTransform transform;
		grasp_template::GraspTemplateParams params;
		viewpoint_pose.header.frame_id = params.frameBase();
		viewpoint_pose.header.stamp = ros::Time::now();
		try
		{
			if (!listener.waitForTransform(params.frameBase(),
					params.frameViewPoint(), ros::Time(0), ros::Duration(1)))
			{
				ROS_ERROR_STREAM("Waiting for transform, from " << params.frameBase()
						<< " to " << params.frameViewPoint() << " timed out.");
				return -1;
			}
			else
			{
				listener.lookupTransform(params.frameBase(),
						params.frameViewPoint(), ros::Time(0), transform);
				viewpoint_pose.pose.position.x = transform.getOrigin().x();
				viewpoint_pose.pose.position.y = transform.getOrigin().y();
				viewpoint_pose.pose.position.z = transform.getOrigin().z();
				viewpoint_pose.pose.orientation.w = transform.getRotation().w();
				viewpoint_pose.pose.orientation.x = transform.getRotation().x();
				viewpoint_pose.pose.orientation.y = transform.getRotation().y();
				viewpoint_pose.pose.orientation.z = transform.getRotation().z();
			}
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
		}
	}

	return viewpoint_pose;
}

void execute_action::executeCallBack(const feature_learning::ExecuteActionGoalConstPtr &goal){

	int action = goal->action_number;
    std::string controller_prefix;

    //Finding out Hand : TODO: Add a failsafe for no number??
    bool right_hand = true;
    if(goal->hand_number == 1)
    	controller_prefix = "Right";
    else{
    	controller_prefix = "Left";
    	right_hand = false;
    }


	char input = 'p';
	ROS_INFO("Note: Only demonstrations with the right arm are recorded.");

	ROS_INFO("Please, free the robot's view on the object "
			"and enter an arbitrary character followed by enter.");
	std::cin >> input;
	// Now first get the topics we were looking for

	geometry_msgs::PoseStamped surface_pose = getSurfacePose();

	// Need to call the manipulation object here and switch the controller stack

	geometry_msgs::PoseStamped viewpoint_pose = getViewpointPose();

	ROS_INFO("Please, place the gripper at the object without displacing "
			"it and enter an arbitrary character followed by enter.");
	std::cin >> input;

	//Now place the hand in gravity comp
	ROS_VERIFY(switch_controller_stack_.switchControllerStack(controller_prefix + "GravityCompensation"));

	ROS_INFO("Please, Verify if hand has been placed in appropriate location "
			"and enter an arbitrary character followed by enter.");
	std::cin >> input;

	if(right_hand)
		manipulation_object_r_.switchControl(ArmInterface::JOINT_CONTROL_);
	else
		manipulation_object_l_.switchControl(ArmInterface::JOINT_CONTROL_);

	geometry_msgs::PoseStamped gripper_pose = getGripperPose();

	ExtractFeatures extract_feature_service;
	extract_feature_service.request.gripper_pose = gripper_pose;
	extract_feature_service.request.view_point = viewpoint_pose;
	extract_feature_service.request.surface_pose = surface_pose.pose;

	while (!extract_feature_client_.waitForExistence(ros::Duration(1.0)) && ros::ok())
	{
		ros::Rate r(1);
		r.sleep();
	}
	if (!extract_feature_client_.call(extract_feature_service))
	{
		ROS_INFO("feature_learning::execute_action: Could not get feature extraction response");
		return false;
	}

	geometry_msgs::Point action_point;

	if(extract_feature_service.response.result == extract_feature_service.response.SUCCESS)
	{
		action_point = extract_feature_service.response.action_location.point;
		result_.success = true;
	}
	else{
		result_.success = false;
	}

	// If action succeeded
	if(result_.success)
	{
		// Now call the controller stack and execute the action
		bool action_succeeded;
		if(right_hand)
			action_succeeded = doActionOnPoint(action_point,action,manipulation_object_r_);
		else
			action_succeeded = doActionOnPoint(action_point,action,manipulation_object_l_);
		if(action_succeeded)
		{
			ROS_INFO("%s: Action Succeeded", getName().c_str ());
			// set the action state to succeeded
			action_server_.setSucceeded(result_);
		}

	}

}

void execute_action::getPushPose(const geometry_msgs::PoseStamped &start_pose, geometry_msgs::PoseStamped &end_pose,
		double direction){

	// Converting the push direction
	tf::Vector3 x_axis(1,0,0);
	tf::Vector3 z_axis(0,0,1);
	tf::Pose start_direction_tf;
	conversions::convert(start_pose.pose,start_direction_tf);

	tf::Vector3 local_z_axis = start_direction_tf.getBasis().inverse()*z_axis;
	tf::Matrix3x3 m(tf::Quaternion(local_z_axis,-direction));
	tf::Transform pose_transform = tf::Transform::getIdentity();
	pose_transform.setBasis(m);

	tf::Pose new_pose = start_direction_tf*pose_transform;

	double push_theta = M_PI/2;
	tf::Vector3 local_push_x_axis = new_pose.getBasis().inverse()*x_axis;
	tf::Matrix3x3 m_push(tf::Quaternion(local_push_x_axis,push_theta));
	tf::Transform push_pose_transform = tf::Transform::getIdentity();
	push_pose_transform.setBasis(m_push);
	tf::Pose new_push_pose = new_pose*push_pose_transform;
	conversions::convert(new_push_pose,end_pose.pose);

}

void execute_action::getGraspPose(const geometry_msgs::PoseStamped &start_pose, geometry_msgs::PoseStamped &end_pose,
		 double direction){

	tf::Vector3 x_axis(1,0,0);
	tf::Vector3 z_axis(0,0,1); // For grasp reorientation
	tf::Pose grasp_pose_tf;
	tf::poseMsgToTF(start_pose.pose,grasp_pose_tf);

	//Now rotate the hand (object_pose) by 180 degrees
	// Computing the yaw
	double theta = M_PI;
	tf::Vector3 local_x_axis = grasp_pose_tf.getBasis().inverse()*x_axis;
	tf::Matrix3x3 m_x(tf::Quaternion(local_x_axis,theta));
	tf::Transform grasp_x_pose_transform = tf::Transform::getIdentity();
	grasp_x_pose_transform.setBasis(m_x);
	tf::Pose new_object_pose = grasp_pose_tf*grasp_x_pose_transform;

	// Now rotate the GRASP Pose in the local axis by z-dir
	// TODO: Use to the local point cloud template based on the max
	// radius of the hand and compute eigen values and align with the direction
	// that does not correspond to the dominant eigen value i.e thinner cloud

	double psi = -direction;
	tf::Vector3 local_z_axis = grasp_pose_tf.getBasis().inverse()*z_axis;
	tf::Matrix3x3 m_z(tf::Quaternion(local_z_axis,psi));
	tf::Transform grasp_z_pose_transform = tf::Transform::getIdentity();
	grasp_z_pose_transform.setBasis(m_z);
	new_object_pose = grasp_pose_tf*grasp_z_pose_transform;
	conversions::convert(new_object_pose,end_pose.pose);



}

bool execute_action::doActionOnPoint(const geometry_msgs::Point &action_point, int action, ArmInterface& manipulation_object){

	// Computing the manipulation pose, i.e rotating Hand around x axis by 90 degrees
	geometry_msgs::PoseStamped manipulation_pose;
	manipulation_pose.header.stamp = ros::Time::now();
	manipulation_pose.pose.position = action_point;

	manipulation_pose.pose.orientation.w = 1;
	manipulation_pose.pose.orientation.x = 0;
	manipulation_pose.pose.orientation.y = 0;
	manipulation_pose.pose.orientation.z = 0;

	double y_direction;

	switch (action){
	case 2: // Push Front
		y_direction = 0.0;
		break;
	case 3: //Push back
		y_direction = M_PI;
		break;
	case 4: //Push Right
		y_direction = M_PI/2;
		break;
	case 5: //Push Left
		y_direction = M_PI/2 + M_PI;
		break;
	default:
		y_direction = INFINITY;
		break;
	}

	if(isinf(y_direction))
		return false;

	getPushPose(manipulation_pose,manipulation_pose,y_direction);

	ROS_INFO("Manipulating at Location");
	if(action == 1){

		// Now computing the GRASP Pose
		geometry_msgs::PoseStamped grasp_pose;
		grasp_pose = manipulation_pose;
		grasp_pose.pose.orientation.w = 1.0;
		grasp_pose.pose.orientation.z = 0.0;
		grasp_pose.pose.orientation.y = 0.0;
		grasp_pose.pose.orientation.x = 0.0;

		getGraspPose(grasp_pose,grasp_pose,y_direction);

		if(graspNode(grasp_pose,manipulation_object))
		{
			ROS_INFO("Grasping Succeeded ");
			return true;
		}
		else{
			ROS_INFO("Pushing Failed too");
			return false;
		}
	}

	if(pushNode(manipulation_pose,y_direction,manipulation_object)){
		ROS_INFO("Pushing Succeeded ");
		return true;
	}
	else{
		ROS_INFO("Pushing Failed too");
		return false;
	}
}




}
