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
#define DEBUG false

namespace feature_learning {

execute_action::execute_action(ros::NodeHandle & nh):
		nh_(nh),nh_priv_("~"),
		action_server_(nh_,"execute_action",boost::bind(&execute_action::executeCallBack,this,_1),false),
		manipulation_object_l_(2),manipulation_object_r_(1),
		poke_object_(),world_state_(){

	// Start the action server
	action_server_.start();
	tabletop_client_ = nh_priv_.serviceClient<tabletop_segmenter::TabletopSegmentation>("/tabletop_segmentation");
	extract_feature_client_ = nh_priv_.serviceClient<ExtractFeatures>("/extract_features_srv");
	pose_publisher_ = nh_priv_.advertise<geometry_msgs::PoseStamped>("/manipulation_pose",5);
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
			return table_pose;
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
					ros::Time(0), ros::Duration(5.0)))
			{
				ROS_ERROR_STREAM("feature_learning::execute_action: Waiting for transform, from " << params.frameBase()
						<< " to " << plan_params.frameGripper() << " timed out.");
				return gripper_pose;
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
				ROS_ERROR_STREAM("feature_learning::execute_action: Waiting for transform, from " << params.frameBase()
						<< " to " << params.frameViewPoint() << " timed out.");
				return viewpoint_pose;
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
    ROS_INFO("Action selected is %d",action);
    //Finding out Hand : TODO: Add a failsafe for no number??
    bool right_hand = true;
    if(goal->hand_number == 1)
    	controller_prefix = "Right";
    else{
    	controller_prefix = "Left";
    	right_hand = false;
    }


	char input = 'p';
	// TODO: Is this true??
	ROS_INFO("feature_learning::execute_action: Note: Only demonstrations with the right arm are recorded.");

	ROS_INFO("feature_learning::execute_action: Please, free the robot's view on the object "
			"and enter an arbitrary character followed by enter.");
	std::cin >> input;
	// Now first get the topics we were looking for

	geometry_msgs::PoseStamped surface_pose = getSurfacePose();

	// Need to call the manipulation object here and switch the controller stack

	geometry_msgs::PoseStamped viewpoint_pose = getViewpointPose();

	ROS_INFO("feature_learning::execute_action: Please, place the gripper at the object without displacing "
			"it and enter an arbitrary character followed by enter.");

	std::cin >> input;
	std::cout<<controller_prefix<<"GravityCompensation"<<" is the controller stack being run"<<std::endl;
	//Now place the hand in gravity comp
	ROS_VERIFY(switch_controller_stack_.switchControllerStack(controller_prefix + "GravityCompensation"));

	std::cin >> input;
	ROS_INFO("feature_learning::execute_action: Please, Verify if hand has been placed in appropriate location and move out of the way "
			"and enter an arbitrary character followed by enter.");
	std::cin >> input;

	ROS_INFO("feature_learning::execute_action: Verifying right hand now %d",right_hand);

	ROS_VERIFY(switch_controller_stack_.switchControllerStack(controller_prefix + "HandJointPDControl"));
	if(right_hand)
	{
		manipulation_object_r_.switchControl(ArmInterface::JOINT_CONTROL_);
	}
	else
	{
		manipulation_object_l_.switchControl(ArmInterface::JOINT_CONTROL_);
	}


	geometry_msgs::PoseStamped gripper_pose = getGripperPose();

	ExtractFeatures extract_feature_service;
	extract_feature_service.request.gripper_pose = gripper_pose;
	extract_feature_service.request.view_point = viewpoint_pose;
	extract_feature_service.request.surface_pose = surface_pose.pose;

	while (!extract_feature_client_.waitForExistence(ros::Duration(10.0)) && ros::ok())
	{
		ros::Rate r(1);
		r.sleep();
	}
	if (!extract_feature_client_.call(extract_feature_service))
	{
		ROS_INFO("feature_learning::execute_action: Could not get feature extraction response");
		result_.success = false;
		action_server_.setAborted(result_);
		return;
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

	bool interrupt;
	char interrupt_choice;
	ROS_INFO_STREAM(" Action Point received from Feature Extraction Service is "<<action_point<<std::endl
			<<" Do you want to continue (y/n): ");
	std::cin>>interrupt_choice;
	if(interrupt_choice == 'n')
	{
		result_.success = false;
	}
	else{

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
				ROS_INFO("feature_learning::execute_action: %s: Action Succeeded", getName().c_str ());
				// Send both hands home
				manipulation_object_l_.switchStackToJointControl();
				manipulation_object_l_.goHome();
				manipulation_object_r_.switchStackToJointControl();
				manipulation_object_r_.goHome();
				// set the action state to succeeded
				ROS_INFO("feature_learning::execute_action: Extract Feature Service and Action succeeded");
				action_server_.setSucceeded(result_);
			}
			else{
				ROS_INFO("feature_learning::execute_action: Extract Feature Service succeeded but execute action failed");
				action_server_.setAborted(result_);
			}
		}
		else{
			ROS_INFO("feature_learning::execute_action: Extract Feature Service failed, check for errors");
			action_server_.setAborted(result_);
		}
	}

}

void execute_action::getPushPose(const geometry_msgs::PoseStamped &start_pose, geometry_msgs::PoseStamped &end_pose,
		double direction){

	// Converting the push direction
	tf::Vector3 y_axis(0,1,0);
	tf::Vector3 x_axis(0,0,1);
	tf::Pose start_direction_tf;
	conversions::convert(start_pose.pose,start_direction_tf);

	double push_theta = M_PI/2;
	tf::Vector3 local_z_axis = start_direction_tf.getBasis().inverse()*y_axis;
	tf::Matrix3x3 m(tf::Quaternion(local_z_axis,-push_theta));
	geometry_msgs::Quaternion initial_quat;
	conversions::convert(tf::Quaternion(local_z_axis,-push_theta),initial_quat);
	ROS_INFO_STREAM(" New Pose quaternion "<< initial_quat<<" Direction "<<-direction);
	tf::Transform pose_transform = tf::Transform::getIdentity();
	pose_transform.setBasis(m);

	start_direction_tf = start_direction_tf*pose_transform;
	geometry_msgs::Pose display_pose;
	conversions::convert(start_direction_tf,display_pose);
	ROS_INFO_STREAM(" New Pose origin "<< display_pose<<std::endl<<" Direction flip in x-axis "<<-direction);

	tf::Vector3 local_push_x_axis = start_direction_tf.getBasis().inverse()*x_axis;
	tf::Matrix3x3 m_push(tf::Quaternion(local_push_x_axis,-direction));
	tf::Transform push_pose_transform = tf::Transform::getIdentity();
	push_pose_transform.setBasis(m_push);
	start_direction_tf = start_direction_tf*push_pose_transform;
	conversions::convert(start_direction_tf,end_pose.pose);

}

void execute_action::getGraspPose(const geometry_msgs::PoseStamped &start_pose, geometry_msgs::PoseStamped &end_pose,
		 double direction){

	tf::Vector3 x_axis(1,0,0);
	tf::Vector3 z_axis(0,0,1); // For grasp reorientation
	tf::Pose grasp_pose_tf;
	tf::poseMsgToTF(start_pose.pose,grasp_pose_tf);

//	//Now rotate the hand (object_pose) by 180 degrees
//	// Computing the yaw
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
//	tf::Pose new_object_pose;
	double psi = -direction;
	tf::Vector3 local_z_axis = new_object_pose.getBasis().inverse()*z_axis;
	tf::Matrix3x3 m_z(tf::Quaternion(local_z_axis,psi));
	tf::Transform grasp_z_pose_transform = tf::Transform::getIdentity();
	grasp_z_pose_transform.setBasis(m_z);
	new_object_pose = new_object_pose*grasp_z_pose_transform;
	conversions::convert(new_object_pose,end_pose.pose);



}

bool execute_action::doActionOnPoint(const geometry_msgs::Point &action_point, int action, ArmInterface& manipulation_object){

	ROS_INFO("feature_learning::execute_action: Starting doAction function");
	// Computing the manipulation pose, i.e rotating Hand around x axis by 90 degrees
	geometry_msgs::PoseStamped manipulation_pose;
	manipulation_pose.header.stamp = ros::Time::now();
	manipulation_pose.pose.position.x = action_point.x;
	manipulation_pose.pose.position.y = action_point.y;
	manipulation_pose.pose.position.z = action_point.z;
	manipulation_pose.header.frame_id = "/BASE";

	manipulation_pose.pose.orientation.w = 1;
	manipulation_pose.pose.orientation.x = 0;
	manipulation_pose.pose.orientation.y = 0;
	manipulation_pose.pose.orientation.z = 0;

	double y_direction;

	// NOTE: Directions calibrated to rotated axis
	switch (action){
	case 2: // Push Front
		y_direction = M_PI/2;
		break;
	case 3: //Push back
		y_direction = M_PI/2 + M_PI;
		break;
	case 4: //Push Right
		y_direction = M_PI;
		break;
	case 5: //Push Left
		y_direction = 0.0;
		break;
	default:
		y_direction = INFINITY;
		break;
	}

	ROS_INFO_STREAM("Selected Action is "<<action<<" Selected direction is "<<y_direction);
	if(isinf(y_direction) && action != 1)
		return false;

	char pose_decision;
	ROS_INFO("Manipulating at Location");
	if(action == 1){

		// Now computing the GRASP Pose
		geometry_msgs::PoseStamped grasp_pose;
		grasp_pose = manipulation_pose;
		grasp_pose.pose.orientation.w = 1.0;
		grasp_pose.pose.orientation.z = 0.0;
		grasp_pose.pose.orientation.y = 0.0;
		grasp_pose.pose.orientation.x = 0.0;

		ROS_INFO_STREAM("Original Grasp Pose "<<grasp_pose.pose);
		getGraspPose(grasp_pose,grasp_pose,0.0);

		pose_publisher_.publish(grasp_pose);
		ROS_INFO_STREAM("Grasp Pose Published "<<grasp_pose.pose);

		ROS_INFO("Do you want to quit now (y/n)? ");
		std::cin>>pose_decision;
		if(pose_decision == 'y')
			return false;


		if(graspNode(grasp_pose,manipulation_object))
		{
			ROS_INFO("Grasping Succeeded ");
			return true;
		}
		else{
			ROS_INFO("Grasping Failed");
			return false;
		}
	}

	ROS_INFO("feature_learning::execute_action: getting push Pose");
	ROS_INFO_STREAM("Original Manipulation Pose "<<manipulation_pose.pose);
	getPushPose(manipulation_pose,manipulation_pose,y_direction);
	ROS_INFO_STREAM("Computed Manipulation Pose "<<manipulation_pose.pose);

	pose_publisher_.publish(manipulation_pose);

	ROS_INFO("Do you want to quit now (y/n)? ");
	std::cin>>pose_decision;
	if(pose_decision == 'y')
		return false;

	if(pushNode(manipulation_pose,y_direction,manipulation_object)){
		ROS_INFO("Pushing Succeeded ");
		return true;
	}
	else{
		ROS_INFO("Pushing Failed too");
		return false;
	}
}

bool execute_action::graspNode(geometry_msgs::PoseStamped push_pose, ArmInterface& manipulation_object){

	ROS_INFO("Converting pose to TF");
	tf::Pose push_tf;
	tf::poseMsgToTF(push_pose.pose,push_tf);
	ROS_INFO("Converting pose complete");

	// Move to a position that is 10cm above the grasp point
	push_tf.getOrigin().setZ(push_tf.getOrigin().getZ() + 0.10);

	// Moving to desired initial pose
	poke_object_.initialize(manipulation_object.getEndeffectorId());
	world_state_.createTable();
	ROS_INFO("Creating table complete");

	// opening hand
	if(manipulation_object.openFingersAndVerify())
	{
		geometry_msgs::Pose poke_pose;
		poke_pose.position = push_pose.pose.position;
		poke_pose.position.z = push_pose.pose.position.z - 0.10;
		poke_pose.orientation = push_pose.pose.orientation; // This would have to be 1,0,0,0
		ROS_INFO("Fingers opened");


		if(!DEBUG){
			//if(manipulation_object.planAndMoveTo(push_tf)){ //
			manipulation_object.switchControl(ArmInterface::CARTESIAN_CONTROL_);
			ROS_INFO_STREAM("Switched to Cartesian Control 1"<<push_pose.pose.position<<" "<<push_pose.pose.orientation);
			if(manipulation_object.cartesian_.moveTo(push_pose.pose,3.0)){ //

				// Move to poke position
				geometry_msgs::Pose poke_position;
				bool poked, is_pose_relative;
				ROS_INFO("Planning and moved to positions");
				manipulation_object.ft_sensor_.calibrate();
				manipulation_object.switchControl(ArmInterface::CARTESIAN_CONTROL_);
				ROS_INFO("Switching controller stack complete");
				if(poke_object_.run(poke_pose,5.0,3.0,poke_position,poked,is_pose_relative))
				{
					geometry_msgs::Pose pick_position;
					pick_position = poke_position;
					pick_position.position.z += 0.05;//Move  5cm above pick point
					ROS_INFO("poke Successful");
					bool success;
					manipulation_object.switchControl(ArmInterface::CARTESIAN_CONTROL_);
					success = manipulation_object.cartesian_.moveTo(pick_position,3.0);
					ROS_INFO("Moving Cartesian");
					if(success){

						// manipulation object closing Hand
						double mypos[] = {0,2.4,2.4,2.4};
						std::vector<double> hand_joint_positions (mypos, mypos + sizeof(mypos) / sizeof(double) );
						double myforces[] = {-0.5,-0.5,-0.5};
						std::vector<double> hand_joint_forces (myforces, myforces + sizeof(myforces) / sizeof(double) );
						success = manipulation_object.hand_joints_.sendPositionsForces(hand_joint_positions,hand_joint_forces,5.0);
						ROS_INFO("Closing Hands");
						if(success)
						{
							// Now checking the joint angles to check if the Hand is completely closed
							manipulation_object.getHandJointPositions(hand_joint_positions);
							double joint_sum = 0;
							for(int i = 1; i< hand_joint_positions.size() ;i++)
								joint_sum+= hand_joint_positions[i];
							if(joint_sum < 6.0)
							{
								bool place_success = false;
								geometry_msgs::Pose place_position;
								place_position.position = push_pose.pose.position;
								place_position.orientation = push_pose.pose.orientation;
								// Dropping from 5 cn above the ground
								place_position.position.z += 0.05;
								// Now move to a position that is 20 cm away
								double move_x_increment;
								if(manipulation_object.getEndeffectorId() == 2)
									move_x_increment = -0.35;
								else
									move_x_increment = 0.35;
								// Move to a position away from the object
								place_position.position.x += move_x_increment;
								ROS_INFO("Planning to place position");
								do{
									ROS_INFO("Trying to place again");
									tf::Pose place_pose_tf;
									tf::poseMsgToTF(place_position,place_pose_tf);
									if(manipulation_object.planAndMoveTo(place_pose_tf))
									{
										manipulation_object.openFingers();
										ROS_INFO("Dropping object");
										place_success = true;
									}
									else
										place_position.position.y += 0.05;

								}while(!place_success);

								return manipulation_object.goHome();
							}
							else
							{
								// Open fingers and go Home
								ROS_WARN("Couldn't Grasp Object!");
								manipulation_object.openFingers();
								manipulation_object.goHome();
								return false;
							}
						}
						else{
							ROS_WARN("Unable to close hands!");
							manipulation_object.openFingers();
							manipulation_object.goHome();
							return false;
						}
					}
					else
						return false;
				}
				else
					return false;
			}
			else
				return false;
		}
		else{

			//if(manipulation_object.planAndMoveTo(push_tf)){
			manipulation_object.switchControl(ArmInterface::CARTESIAN_CONTROL_);
			ROS_INFO_STREAM("Switched to Cartesian Control 2"<<push_pose.pose.position<<" "<<push_pose.pose.orientation);
			if(manipulation_object.cartesian_.moveTo(push_pose.pose,3.0)){ //

				geometry_msgs::Pose pick_position;
				//TODO: Do something smarter
				pick_position = push_pose.pose;
				pick_position.position.z += 0.05; // Go 5cm away above the body
				bool success;
				manipulation_object.switchControl(ArmInterface::CARTESIAN_CONTROL_);
				success = manipulation_object.cartesian_.moveTo(pick_position,3.0);

				// Now do a blind Grasp

				if(success){

					// manipulation object closing Hand
					double mypos[] = {0,2.4,2.4,2.4}; // Dont alter the spread just the other joint angles
					std::vector<double> hand_joint_positions (mypos, mypos + sizeof(mypos) / sizeof(double) );
					double myforces[] = {-0.5,-0.5,-0.5};
					std::vector<double> hand_joint_forces (myforces, myforces + sizeof(myforces) / sizeof(double) );
					success = manipulation_object.hand_joints_.sendPositionsForces(hand_joint_positions,hand_joint_forces,5.0);

					if(success)
					{
						// Now checking the joint angles to check if the Hand is completely closed
						manipulation_object.getHandJointPositions(hand_joint_positions);
						double joint_sum = 0;
						for(int i = 1; i< hand_joint_positions.size() ;i++)
							joint_sum+= hand_joint_positions[i];
						if(joint_sum < 6.0)
						{
							bool place_success = false;
							geometry_msgs::Pose place_position;
							place_position.position = push_pose.pose.position;
							place_position.orientation = push_pose.pose.orientation;
							// Now move to a position that is 20 cm away
							double move_x_increment;
							if(manipulation_object.getEndeffectorId() == 2)
								move_x_increment = -0.25;
							else
								move_x_increment = 0.25;
							// Move to a position away from the object
							place_position.position.x += move_x_increment;
							do{
								tf::Pose place_pose_tf;
								tf::poseMsgToTF(place_position,place_pose_tf);
								if(manipulation_object.planAndMoveTo(place_pose_tf))
								{
									manipulation_object.openFingers();
									place_success = true;
								}
								else
									place_position.position.y += 0.05;

							}while(!place_success);

							return manipulation_object.goHome();
						}
						else
						{
							// Open fingers and go Home
							ROS_WARN("Couldn't Grasp Object!");
							manipulation_object.openFingers();
							return manipulation_object.goHome();
						}
					}
					else{
						ROS_WARN("Unable to close hands!");
						manipulation_object.openFingers();
						manipulation_object.goHome();
						return false;
					}
				}
				else
					return false;
			}
			else
				return false;
		}
	}
	else{
		ROS_WARN("Unable to open hands!");
		return false;
	}
}

bool execute_action::pushNode(geometry_msgs::PoseStamped push_pose, double y_dir, ArmInterface& manipulation_object){

	tf::Pose push_tf;
	tf::poseMsgToTF(push_pose.pose,push_tf);
	manipulation_object.goHome();
	// Move to a position that is 10cm behind and 10cm above the desired location
	push_tf.getOrigin().setY(push_tf.getOrigin().getY() - 0.10);
	push_tf.getOrigin().setZ(push_tf.getOrigin().getZ() + 0.10);

	manipulation_object.goHome(); // Just to go home between runs

	poke_object_.initialize(manipulation_object.getEndeffectorId());
	// Moving to desired initial pose
    world_state_.createTable();


	geometry_msgs::Pose poke_pose;
	poke_pose.position = push_pose.pose.position;
	poke_pose.position.z = push_pose.pose.position.z - 0.10;
	poke_pose.orientation = push_pose.pose.orientation;

	if(!DEBUG){

		manipulation_object.switchControl(ArmInterface::CARTESIAN_CONTROL_);
		ROS_INFO_STREAM("Switched to Cartesian Control 3"<<push_pose.pose.position<<" "<<push_pose.pose.orientation);
		if(manipulation_object.cartesian_.moveTo(push_pose.pose,3.0)){ //
		//if(manipulation_object.planAndMoveTo(push_tf)){

			// Remove comment when running on robot
			geometry_msgs::Pose poke_position;
			bool poked, is_pose_relative;
			manipulation_object.ft_sensor_.calibrate();
			manipulation_object.switchControl(ArmInterface::CARTESIAN_CONTROL_);
			if(poke_object_.run(poke_pose,5.0,3.0,poke_position,poked,is_pose_relative))
			{
				geometry_msgs::Pose push_position;
				push_position = poke_position;
				//y dir vector is [sin(t),cos(t),0]

				if(y_dir == (M_PI/2))
					push_position.position.y += 0.20;

				if(y_dir == (M_PI/2 + M_PI)) //Push back
					push_position.position.y -= 0.10;

				if(y_dir == M_PI)
					push_position.position.x += 0.20; //Push Right

				if(y_dir == 0.0) // Push Left
					push_position.position.x -= 0.20;

				//				push_position.position.y += 0.20*(cos(y_dir)); // Push it 20cm away from the body
				//
				//				if(manipulation_object.getEndeffectorId() == 2)
				//					push_position.position.x -= 0.20*(sin(y_dir)); // Push it 20cm away from the body
				//				else
				//					push_position.position.x += 0.20*(sin(y_dir)); // Push it 20cm away from the body
				//
				bool success;
				manipulation_object.switchControl(ArmInterface::CARTESIAN_CONTROL_);
				success = manipulation_object.cartesian_.moveTo(push_position,3.0);

				if(success)
					return manipulation_object.goHome();
				else
					return false;
			}
			else
				return false;
		}
		else
			return false;
	}
	else{
		manipulation_object.switchControl(ArmInterface::CARTESIAN_CONTROL_);
		ROS_INFO_STREAM("Switched to Cartesian Control 4"<<push_pose.pose.position<<" "<<push_pose.pose.orientation);ROS_INFO_STREAM("Switched to Cartesian Control"<<push_pose.pose.position<<" "<<push_pose.pose.orientation);
		if(manipulation_object.cartesian_.moveTo(push_pose.pose,3.0)){
		//if(manipulation_object.planAndMoveTo(push_tf)){

			geometry_msgs::Pose push_position;
			//TODO: Do something smarter
			push_position = push_pose.pose;
			push_position.position.y += 0.05; // Push it 5cm away from the body
			bool success;
			manipulation_object.switchControl(ArmInterface::CARTESIAN_CONTROL_);
			success = manipulation_object.cartesian_.moveTo(push_position,3.0);

			if(success)
				return manipulation_object.goHome();
			else
				return false;
		}
		else
			return false;
	}
}

}

// Main function to run the feature extraction action server
int main(int argc, char** argv){

	ros::init (argc, argv, "execute_action");
	arm_controller_interface::init();
	ros::NodeHandle nh;
	ros::MultiThreadedSpinner mts(0);
	feature_learning::execute_action action (nh);
	mts.spin ();
}


