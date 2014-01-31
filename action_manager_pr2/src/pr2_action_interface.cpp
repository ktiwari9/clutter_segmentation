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
 * \author Bharath Sankaran and Christian Bersch
 *
 * @b control manager for pr2 behaviors for clutter segmentation
 *
 */

#include "action_manager_pr2/pr2_action_interface.hpp"
#include "action_manager_pr2/ReturnJointStates.h"
#include "usc_utilities/assert.h"

namespace action_manager_pr2{

action_manager::action_manager(ros::NodeHandle & nh, const std::string action_name):
			nh_(nh), as_(nh,action_name, boost::bind(&action_manager::execute,this,_1),false), nh_priv_("~"),
			marker_("/rviz_control"),action_name_(action_name){

	nh_priv_.param<std::string>("gripper_r_server",gripper_r_srv_,std::string("r_gripper_controller/gripper_action"));
	nh_priv_.param<std::string>("gripper_l_server",gripper_l_srv_,std::string("l_gripper_controller/gripper_action"));
	nh_priv_.param<std::string>("head_server",head_srv_,std::string("/head_traj_controller/point_head_action"));
	nh_priv_.param<std::string>("head_pointing_frame",head_point_frame_,std::string("head_mount_kinect_depth_optical_frame"));
	nh_priv_.param<std::string>("arm_r_server",arm_r_srv_,std::string("move_right_arm"));
	nh_priv_.param<std::string>("arm_l_server",arm_l_srv_,std::string("move_left_arm"));
	nh_priv_.param<std::string>("right_joint_server",r_joint_srv_,std::string("r_arm_controller/joint_trajectory_action"));
	nh_priv_.param<std::string>("left_joint_server",l_joint_srv_,std::string("l_arm_controller/joint_trajectory_action"));
	nh_priv_.param<std::string>("right_ik_service",r_ik_service_name_,std::string("pr2_right_arm_kinematics/get_ik"));
	nh_priv_.param<std::string>("left_ik_service",l_ik_service_name_,std::string("pr2_left_arm_kinematics/get_ik"));
	nh_priv_.param<std::string>("joint_states_service",joint_states_service_,std::string("return_joint_states"));


	ROS_INFO("action_manager::pr2_action_interface: Initializing clients");
	//Initialize the client for the Action interface to the gripper controllers
	gripper_r_client_ = new GripperClient(gripper_r_srv_,true);
	gripper_l_client_ = new GripperClient(gripper_l_srv_,true);


	//Initialize the client for the Action interface to the head controller
	point_head_client_ = new PointHeadClient(head_srv_, true);

	//Initialize the client for the Action interface to the arm controllers
	arm_r_client_ = new MoveArmClient(arm_r_srv_, true);
	arm_l_client_ = new MoveArmClient(arm_l_srv_, true);

	//Initialize the client for the Action interface to the arm controllers
	r_traj_client_ = new TrajectoryClient(r_joint_srv_, true);
	l_traj_client_ = new TrajectoryClient(l_joint_srv_, true);

	ROS_INFO("action_manager::pr2_action_interface: Waiting for servers to come up");
	//wait for head controller action server to come up
	while(!point_head_client_->waitForServer(ros::Duration(5.0))){
		ROS_INFO("action_manager::pr2_action_interface: Waiting for the point_head_action server to come up");
	}
	//wait for the gripper action server to come up
	while(!gripper_r_client_->waitForServer(ros::Duration(5.0))){
		ROS_INFO("action_manager::pr2_action_interface: Waiting for the r_gripper_controller/gripper_action action server to come up");
	}

	//wait for the gripper action server to come up
	while(!gripper_l_client_->waitForServer(ros::Duration(5.0))){
		ROS_INFO("action_manager::pr2_action_interface: Waiting for the r_gripper_controller/gripper_action action server to come up");
	}

	//wait for the gripper action server to come up
	while(!arm_r_client_->waitForServer(ros::Duration(5.0))){
		ROS_INFO("action_manager::pr2_action_interface: Waiting for the move_right_arm action server to come up");
	}

	//wait for the gripper action server to come up
	while(!arm_l_client_->waitForServer(ros::Duration(5.0))){
		ROS_INFO("action_manager::pr2_action_interface: Waiting for the move_left_arm action server to come up");
	}

	// wait for action server to come up
	while(!r_traj_client_->waitForServer(ros::Duration(5.0))){
		ROS_INFO("action_manager::pr2_action_interface: Waiting for the right joint_trajectory_action server");
	}

	// wait for action server to come up
	while(!l_traj_client_->waitForServer(ros::Duration(5.0))){
		ROS_INFO("action_manager::pr2_action_interface: Waiting for the left joint_trajectory_action server");
	}

	// Staring the action server
	ROS_INFO("action_manager::pr2_action_interface: Starting action server");
	as_.start();
}

action_manager::~action_manager(){

	delete gripper_r_client_; delete gripper_l_client_; delete point_head_client_;
	delete arm_r_client_; delete arm_l_client_; delete r_traj_client_; delete l_traj_client_;
}

void action_manager::execute(const action_manager_pr2::ControllerGoalConstPtr& goal){

	ROS_INFO("action_manager::pr2_action_interface: Received goal to control %d ",goal->controller.target);

	bool success = false;
	int action, direction, arm, gripper;
	std::string frame_id;
	geometry_msgs::PointStamped target_point;
	geometry_msgs::PoseStamped start_pose, end_pose;


	switch(goal->controller.target){

	case (action_manager_msgs::Controller::HEAD):

    				ROS_INFO("action_manager::pr2_action_interface: Controlling head in %s frame",goal->controller.head.frame_id.c_str());
	target_point.point = goal->controller.head.pose.position;
	target_point.header.frame_id = goal->controller.head.frame_id;
	target_point.header.stamp = goal->controller.header.stamp;
	frame_id = "high_def_frame";
	action = goal->controller.head.action;
	success = controlHead(frame_id,target_point,action);

	break;

	case (action_manager_msgs::Controller::GRIPPER):

					ROS_INFO("action_manager::pr2_action_interface: Controlling gripper");
	gripper = goal->controller.gripper.gripper; action = goal->controller.gripper.action;
	success = controlGripper(gripper,action);
	break;

	case (action_manager_msgs::Controller::ARM):

					ROS_INFO("action_manager::pr2_action_interface: Controlling arm");
	start_pose.pose = goal->controller.arm.start_pose;
	start_pose.header.frame_id = goal->controller.arm.frame_id;
	start_pose.header.stamp = goal->controller.header.stamp;

	end_pose.pose = goal->controller.arm.end_pose;
	end_pose.header.frame_id = goal->controller.arm.frame_id;
	end_pose.header.stamp = goal->controller.header.stamp;
	frame_id = goal->controller.arm.frame_id;

	action = goal->controller.arm.action;
	arm = goal->controller.arm.arm;
	direction = goal->controller.arm.direction;
	ROS_INFO("action_manager::pr2_action_interface: Start Pose Frame id: %s",goal->controller.arm.frame_id.c_str());

	success = controlArm(start_pose,end_pose,frame_id,arm,action,direction);
	break;

	default:
		ROS_INFO("action_manager::pr2_action_interface: Incorrect target choice"); success = false; break;
	}

	if(as_.isPreemptRequested() || !ros::ok()){

		ROS_INFO("action_manager::pr2_action_interface: %s Preempted ",action_name_.c_str());
		as_.setPreempted();
		success = false;
		return;
	}
	if(success)
		result_.result  = result_.SUCCEEDED;
	else
		result_.result  = result_.FAILED;

	as_.setSucceeded(result_);
}

bool action_manager::controlArm(const geometry_msgs::PoseStamped& start_pose, const geometry_msgs::PoseStamped& end_pose, const std::string& frame_id, int arm, int action, int direction){

	bool success;

	switch(action){

	case(0):
					success = tuck(arm); break;

	case(1):
					success = stretch(arm);	break;

	case(2):
					success = moveToSide(arm); break;

	case(3):
					success = goZero(); break;

	case(4):
					frame_id_ = frame_id;
	success = graspPlaceAction(start_pose,end_pose); break;

	case(5):
            		frame_id_ = frame_id;
	success = pushAction(start_pose, static_cast<approach_direction_t>(direction)); break;

	default:
		success = false; break;

	}

	return success;
}

std::vector<std::string> action_manager::getJointNames(bool right){

	std::string armside_str;
	if(right)
		armside_str = 'r';
	else
		armside_str = 'l';

	std::vector<std::string> local_joint_names;
	local_joint_names.reserve(7);
	local_joint_names.push_back(armside_str +"_shoulder_pan_joint");
	local_joint_names.push_back(armside_str +"_shoulder_lift_joint");
	local_joint_names.push_back(armside_str +"_upper_arm_roll_joint");
	local_joint_names.push_back(armside_str +"_elbow_flex_joint");
	local_joint_names.push_back(armside_str +"_forearm_roll_joint");
	local_joint_names.push_back(armside_str +"_wrist_flex_joint");
	local_joint_names.push_back(armside_str +"_wrist_roll_joint");

	return local_joint_names;
}

bool action_manager::controlGripper(int hand, int goal){

	pr2_controllers_msgs::Pr2GripperCommandGoal command;

	if(goal)// 1 for close
	{
		command.command.position = 0.0;
		command.command.max_effort = 50.0; // close gently
	}
	else // for open
	{
		command.command.position = 0.08;
		command.command.max_effort = -1.0; // close gently
	}

	if(hand) // 1 for right gripper
	{

		gripper_r_client_->sendGoal(command);
		gripper_r_client_->waitForResult();
		if(gripper_r_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("action_manager::pr2_action_interface: The right gripper actuated!");
			return true;
		}
		else{
			ROS_INFO("action_manager::pr2_action_interface: The right gripper failed to actuate.");
			return false;
		}
	}
	else{
		gripper_l_client_->sendGoal(command);
		gripper_l_client_->waitForResult();
		if(gripper_l_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("action_manager::pr2_action_interface: The left gripper actuated!");
			return true;
		}
		else{
			ROS_INFO("action_manager::pr2_action_interface: The left gripper failed to actuate.");
			return false;
		}
	}
}

bool action_manager::controlHead(const std::string& pointing_frame_id, const geometry_msgs::PointStamped& target_point, int action){

	pr2_controllers_msgs::PointHeadGoal goal;
	goal.target = target_point;
	goal.pointing_frame = pointing_frame_id;
	//and go no faster than 1 rad/s
	goal.max_velocity = 1.0;
	//and go no faster than 1 rad/s
	goal.max_velocity = 1.0;


	if(action) // tracking action
		point_head_client_->sendGoal(goal);
	else
	{ // Point head action
		//take at least 0.5 seconds to get there
		goal.min_duration = ros::Duration(0.5);
		point_head_client_->sendGoal(goal);
		point_head_client_->waitForResult();


		if(point_head_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("action_manager::pr2_action_interface: Move Head success");
			return true;
		}
		else
		{
			ROS_INFO("action_manager::pr2_action_interface: Move Head failed");
			return false;
		}
	}
	return true;
}

bool action_manager::updateJointStates(bool right){

	action_manager_pr2::ReturnJointStates req;
	req.request.name = getJointNames(right);

	if(!ros::service::waitForService(joint_states_service_, ros::Duration(5.0))){
		ROS_ERROR("action_manager::pr2_action_interface: Could not contact : %s. Did you run joint_state_listner?",joint_states_service_.c_str());
		return false;
	}
	if (!ros::service::call(joint_states_service_,req)){
		ROS_ERROR("action_manager::pr2_action_interface: service call failed");
		return false;
	}

	current_joint_angles_=  req.response.position;
	return true;

}

bool action_manager::goToJointPosWithCollisionChecking(const std::vector<double>& positions, double max_time, bool wait, bool right){

	if(right){
		if (!arm_r_client_){
			ROS_ERROR("action_manager::pr2_action_interface: collision checking arm server has not been started");
			return false;
		}

	}
	else{
		if (!arm_l_client_){
			ROS_ERROR("action_manager::pr2_action_interface: collision checking arm server has not been started");
			return false;
		}
	}

	arm_navigation_msgs::MoveArmGoal goalB;

	if(right)
		group_name_= "right_arm";
	else
		group_name_= "left_arm";

	joint_names_ = getJointNames(right);
	goalB.motion_plan_request.group_name = group_name_;
	goalB.motion_plan_request.num_planning_attempts = 1;
	goalB.motion_plan_request.allowed_planning_time = ros::Duration(max_time);

	goalB.motion_plan_request.planner_id= std::string("");
	goalB.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
	goalB.motion_plan_request.goal_constraints.joint_constraints.resize(7);

	for (unsigned int i = 0 ; i < goalB.motion_plan_request.goal_constraints.joint_constraints.size(); ++i)
	{
		goalB.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = joint_names_[i];
		goalB.motion_plan_request.goal_constraints.joint_constraints[i].position = positions[i];
		goalB.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
		goalB.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
	}


	bool finished_within_time = false;
	bool success = false;

	if(right){
		arm_r_client_->sendGoal(goalB);
		finished_within_time = arm_r_client_->waitForResult(ros::Duration(5*max_time));
		if (!finished_within_time)
		{
			arm_r_client_->cancelGoal();
			ROS_INFO("action_manager::pr2_action_interface: Timed out achieving JointPos goal");
		}
		else
		{
			actionlib::SimpleClientGoalState state = arm_r_client_->getState();
			success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
			if(success)
				ROS_INFO("action_manager::pr2_action_interface: Action finished: %s",state.toString().c_str());
			else
				ROS_INFO("action_manager::pr2_action_interface: Action failed: %s",state.toString().c_str());
		}
	}
	else{

		arm_l_client_->sendGoal(goalB);
		finished_within_time = arm_l_client_->waitForResult(ros::Duration(5*max_time));
		if (!finished_within_time)
		{
			arm_l_client_->cancelGoal();
			ROS_INFO("action_manager::pr2_action_interface: Timed out achieving  JointPos goal");
		}
		else
		{
			actionlib::SimpleClientGoalState state = arm_l_client_->getState();
			success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
			if(success)
				ROS_INFO("action_manager::pr2_action_interface: Action finished: %s",state.toString().c_str());
			else
				ROS_INFO("action_manager::pr2_action_interface: Action failed: %s",state.toString().c_str());
		}

	}


	return finished_within_time && success;
}

std::vector<double> action_manager::getCurrentJointAngles(bool right){

	updateJointStates(right);
	return current_joint_angles_;
}

bool action_manager::rotateWrist(double radians, double wrist_speed, bool wait, bool right){


	std::vector<double> new_pos = getCurrentJointAngles(right);
	new_pos[6] += radians;
	return goToJointPos(new_pos, std::abs(radians) / 2 / 3.14 * wrist_speed, wait,right);
}

bool action_manager::goToJointPos (const double* positions , int num_positions, double max_time , bool wait, bool right){
	std::vector<double> pos_vec (positions, positions + 7 * num_positions);
	return goToJointPos(pos_vec, max_time, wait, right);
}

bool action_manager::goToJointPos (const std::vector<double>& positions , double max_time, bool wait, bool right){

	if (positions.size() % 7 != 0){
		ROS_ERROR("action_manager::pr2_action_interface: you must specify 7 (or a multiple thereof) for the arm joint positions");
	}
	//our goal variable
	pr2_controllers_msgs::JointTrajectoryGoal goal;
	goal.trajectory.joint_names = getJointNames(right);
	unsigned int num_waypoints = positions.size() / 7;
	goal.trajectory.points.resize(num_waypoints);
	std::vector<double>::const_iterator it = positions.begin();
	std::vector<double>::const_iterator it_end = positions.begin() + 7;

	for (unsigned int i=0; i <num_waypoints; i++){

		goal.trajectory.points[i].positions.insert(goal.trajectory.points[i].positions.begin(), it, it_end);
		goal.trajectory.points[i].velocities.resize(7);
		goal.trajectory.points[i].time_from_start = ros::Duration( (i+1) * max_time / num_waypoints);
		it =it_end;
		it_end+=7;
	}

	// When to start the trajectory: 1s from now
	goal.trajectory.header.stamp = ros::Time::now();// + ros::Duration(1.0);
	ROS_INFO("action_manager::pr2_action_interface: sending goal");
	if(right)
		r_traj_client_->sendGoal(goal);
	else
		l_traj_client_->sendGoal(goal);

	bool finished_before_timeout = false;
	if (wait){
		if(right)
			finished_before_timeout = r_traj_client_->waitForResult(ros::Duration(3*max_time));
		else
			finished_before_timeout = l_traj_client_->waitForResult(ros::Duration(3*max_time));

		if (finished_before_timeout)
		{
			if(right){
				actionlib::SimpleClientGoalState state = r_traj_client_->getState();
				ROS_INFO("action_manager::pr2_action_interface: move to joint pos Action finished: %s",state.toString().c_str());
			}
			else{
				actionlib::SimpleClientGoalState state = l_traj_client_->getState();
				ROS_INFO("action_manager::pr2_action_interface: move to joint pos Action finished: %s",state.toString().c_str());
			}

		}
		else
			ROS_INFO("action_manager::pr2_action_interface: move to joint pos Action did not finish before the time out.");
	}

	return finished_before_timeout;
}

bool action_manager::moveWristRollLinktoPose(const tf::StampedTransform& tf,  double max_time, bool wait, std::vector<double>* ik_seed_pos, bool right){

	geometry_msgs::PoseStamped pose;
	tf::Stamped<tf::Pose> tf_pose(tf,tf.stamp_, tf.frame_id_);

	tf::poseStampedTFToMsg(tf_pose,pose);
	return moveWristRollLinktoPose(pose, max_time, wait, ik_seed_pos, right);
}

bool action_manager::moveWristRollLinktoPose(const geometry_msgs::PoseStamped& pose,  double max_time, bool wait, std::vector<double>* ik_seed_pos, bool right){

	std::vector<double> joint_angles;
	if (!getIK(pose ,joint_angles , ik_seed_pos,right)){
		return false;
	}

	return goToJointPos(joint_angles, max_time, wait,right);
}


bool action_manager::moveWristRollLinktoPoseWithCollisionChecking(const tf::StampedTransform& tf,  double max_time, bool wait, std::string planner, bool right){
	geometry_msgs::PoseStamped pose;
	tf::Stamped<tf::Pose> tf_pose(tf,tf.stamp_, tf.frame_id_);

	tf::poseStampedTFToMsg(tf_pose,pose);
	return moveWristRollLinktoPoseWithCollisionChecking(pose, max_time, wait, planner, right);
}

bool action_manager::moveWristRollLinktoPoseWithCollisionChecking(const geometry_msgs::PoseStamped& pose,  double max_time , bool wait, std::string planner, bool right){


	if(right){
		if (!arm_r_client_){
			ROS_ERROR("action_manager::pr2_action_interface: collision checking arm server has not been started");
			return false;
		}
	}
	else{
		if (!arm_l_client_){
			ROS_ERROR("action_manager::pr2_action_interface: collision checking arm server has not been started");
			return false;
		}
	}

	arm_navigation_msgs::MoveArmGoal goalA;

	if(right)
		group_name_= "right_arm";
	else
		group_name_= "left_arm";

	goalA.motion_plan_request.group_name = group_name_;
	goalA.motion_plan_request.num_planning_attempts = 1;
	goalA.motion_plan_request.planner_id = std::string("");
	if (planner == "chomp"){
		ROS_INFO("action_manager::pr2_action_interface: using chomp planner");
		goalA.planner_service_name = std::string("/chomp_planner_longrange/plan_path");
	}else{
		ROS_INFO("action_manager::pr2_action_interface: using ompl planner");
		goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
	}



	goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

	arm_navigation_msgs::SimplePoseConstraint desired_pose;

	desired_pose.header.frame_id = pose.header.frame_id;

	if(right)
		desired_pose.link_name = "r_wrist_roll_link";
	else
		desired_pose.link_name = "l_wrist_roll_link";

	desired_pose.pose = pose.pose;

	desired_pose.absolute_position_tolerance.x = 0.02;
	desired_pose.absolute_position_tolerance.y = 0.02;
	desired_pose.absolute_position_tolerance.z = 0.02;

	desired_pose.absolute_roll_tolerance = 0.04;
	desired_pose.absolute_pitch_tolerance = 0.04;
	desired_pose.absolute_yaw_tolerance = 0.04;

	arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);


	bool finished_within_time = false;
	bool success = false;

	if(right){
		arm_r_client_->sendGoal(goalA);
		finished_within_time = arm_r_client_->waitForResult(ros::Duration(40.0));
		if (!finished_within_time)
			ROS_INFO("action_manager::pr2_action_interface: Action did not finish before time out");
		else
		{
			actionlib::SimpleClientGoalState state = arm_r_client_->getState();
			success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
			ROS_INFO("action_manager::pr2_action_interface: Action finished: %s",state.toString().c_str());
		}
	}
	else{
		arm_l_client_->sendGoal(goalA);
		finished_within_time = arm_l_client_->waitForResult(ros::Duration(40.0));
		if (!finished_within_time)
			ROS_INFO("action_manager::pr2_action_interface: Action did not finish before time out");
		else
		{
			actionlib::SimpleClientGoalState state = arm_l_client_->getState();
			success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
			ROS_INFO("action_manager::pr2_action_interface: Action finished: %s",state.toString().c_str());
		}


	}
	return finished_within_time && success;
}

bool action_manager::getIK(const geometry_msgs::PoseStamped& pose, std::vector<double>& joint_angles, std::vector<double>* ik_seed_pos, bool right){


	if (ik_seed_pos){
		ik_service_client_.request.ik_request.ik_seed_state.joint_state.position = *ik_seed_pos;
	}else{
		ik_service_client_.request.ik_request.ik_seed_state.joint_state.position = getCurrentJointAngles();

	}

	ik_service_client_.request.timeout = ros::Duration(5.0);
	ik_service_client_.request.ik_request.ik_seed_state.joint_state.name = getJointNames(right);
	ik_service_client_.request.ik_request.pose_stamped = pose;

	std::string ik_service_name;
	if(right){
		ik_service_client_.request.ik_request.ik_link_name = "r_wrist_roll_link";
		ik_service_name = r_ik_service_name_;
	}
	else{
		ik_service_client_.request.ik_request.ik_link_name = "l_wrist_roll_link";
		ik_service_name = l_ik_service_name_;
	}


	if (!ros::service::waitForService(ik_service_name, ros::Duration(5.0))){
		ROS_ERROR("action_manager::pr2_action_interface: Could not find ik server %s", ik_service_name.c_str());
		return false;
	}

	//calling IK service
	if (!ros::service::call(ik_service_name, ik_service_client_)) {

		ROS_ERROR("action_manager::pr2_action_interface: ik_service_call failed");
		return false;
	}

	if (ik_service_client_.response.error_code.val != 1){
		ROS_ERROR("action_manager::pr2_action_interface: Could not get valid IK: error code %d", ik_service_client_.response.error_code.val);
		return false;
	}

	joint_angles = ik_service_client_.response.solution.joint_state.position;
	return true;

}

bool action_manager::isAtPos(const std::vector<double>& pos_vec, bool right){
	const double epsilon= 0.1;
	updateJointStates(right);
	int pos = pos_vec.size() - 7;
	for (int i=0; i<7; i++){
		printf ("action_manager::pr2_action_interface: current pos %f pos_vec %f \n", current_joint_angles_[i], pos_vec[pos + i] );
		if (std::abs(current_joint_angles_[i] - pos_vec[pos + i]) > epsilon)
			return false;
	}
	return true;
}

bool action_manager::goZero(){

	bool success;
	success = moveToSide(true);
	if(success)
		success = moveToSide(false);
	return success;
}

bool action_manager::moveToSide(bool right){

	if(right)
		ROS_INFO(" action_manager::pr2_action_interface: zeroing right arm ");
	else
		ROS_INFO(" action_manager::pr2_action_interface: zeroing left arm ");

	std::vector<double> tuck_pos_vec;
	if (right){
		double tuck_pos[] = {-2.135, 0.803, -1.732, -1.905, -2.369, -1.680, 1.398};
		tuck_pos_vec.insert(tuck_pos_vec.begin(),tuck_pos, tuck_pos+7);
		//location = [0.05, -0.65, -0.05] 'torso_lift_link'
	}else{
		double tuck_pos[] = {2.135, 0.803, 1.732, -1.905, 2.369, -1.680, 1.398};
		tuck_pos_vec.insert(tuck_pos_vec.begin(),tuck_pos, tuck_pos+7);
		//location = [0.05, 0.65, -0.05] 'torso_lift_link'
	}

	if (!isAtPos(tuck_pos_vec,right)){
		return goToJointPos(tuck_pos_vec,3.0,true,right);
	}

	if(right)
		ROS_INFO(" action_manager::pr2_action_interface: right arm is in zero pos ");
	else
		ROS_INFO(" action_manager::pr2_action_interface: left arm is in zero pos ");

	return true;

}


bool action_manager::tuck(bool right){

	if(right)
		ROS_INFO(" action_manager::pr2_action_interface: tucking right arm ");
	else
		ROS_INFO(" action_manager::pr2_action_interface: tucking left arm ");

	std::vector<double> tuck_pos_vec;
	if (right){
		double tuck_pos[] = { -0.4,0.0,0.0,-2.25,0.0,0.0,0.0, -0.01,1.35,-1.92,-1.68, 1.35,-0.18,0.31};
		tuck_pos_vec.insert(tuck_pos_vec.begin(),tuck_pos, tuck_pos+14);
	}else{
		double tuck_pos[] = {   0.4,0.0,0.0,-2.25,0.0,0.0,0.0, -0.05,1.31,1.38,-2.06,1.69,-2.02,2.44};
		tuck_pos_vec.insert(tuck_pos_vec.begin(),tuck_pos, tuck_pos+14);
	}

	if (!isAtPos(tuck_pos_vec,right)){
		return goToJointPos(tuck_pos_vec,3.0,true,right);
	}

	if(right)
		ROS_INFO(" action_manager::pr2_action_interface: right arm is already in tucked pos ");
	else
		ROS_INFO(" action_manager::pr2_action_interface: left arm is already in tucked pos ");

	return true;

}

bool action_manager::stretch(bool right){

	if(right)
		ROS_INFO(" action_manager::pr2_action_interface: stretching right arm ");
	else
		ROS_INFO(" action_manager::pr2_action_interface: stretching left arm ");

	std::vector<double> tuck_pos_vec;

	if (right){
		double tuck_pos[] = {-1.634, -0.039, -0.324, -0.131, 31.779, 0.004, 24.986};
		tuck_pos_vec.insert(tuck_pos_vec.begin(),tuck_pos, tuck_pos+7);
	}else{
		double tuck_pos[] = {1.613, -0.105, 0.336, -0.033, -6.747, 0.014, 0.295};
		tuck_pos_vec.insert(tuck_pos_vec.begin(),tuck_pos, tuck_pos+7);
	}
	if (!isAtPos(tuck_pos_vec)){
		return goToJointPos(tuck_pos_vec,3.0,true,right);
	}

	if(right)
		ROS_INFO(" action_manager::pr2_action_interface: right arm is already in stretched pos ");
	else
		ROS_INFO(" action_manager::pr2_action_interface: left arm is already in stretched pos ");

	return true;
}


tf::StampedTransform action_manager::gripperToWrist(const tf::StampedTransform& pose){

	tf::Vector3 offset(gripper_length_,0,0);
	tf::Transform rot(pose.getRotation());
	tf::StampedTransform out( tf::Transform(pose.getRotation(), pose.getOrigin() - rot * offset) , pose.stamp_, pose.frame_id_, pose.child_frame_id_);

	return out;
}

tf::StampedTransform action_manager::wristToGripper(const tf::StampedTransform& pose){
	tf::Vector3 offset(gripper_length_,0,0);
	tf::Transform rot(pose.getRotation());
	tf::StampedTransform out( tf::Transform(pose.getRotation(), pose.getOrigin() + rot * offset) , pose.stamp_, pose.frame_id_, pose.child_frame_id_);
	return out;
}


tf::StampedTransform action_manager::makePose(const tf::Vector3& position, std::string frame_id, approach_direction_t approach){


	tf::StampedTransform tf(tf::Transform(tf::Quaternion(0,0,0,1), position), ros::Time(), frame_id, "/doesntmatter");
	tf::TransformListener tf_listener;
	tf::StampedTransform tf_sourceframe_in_baselink;

	tf_listener.waitForTransform("/base_link", tf.frame_id_, ros::Time(0), ros::Duration(0.5));
	tf_listener.lookupTransform("/base_link", tf.frame_id_, ros::Time(), tf_sourceframe_in_baselink);
	tf::StampedTransform tf_pose_in_baselink (tf::Transform(tf_sourceframe_in_baselink * tf), tf.stamp_, "/base_link", "/doesntmatter") ;

	switch (approach){

	case (FRONTAL):
					tf_pose_in_baselink.setRotation(tf::Quaternion( 0, 0 , 0, 1)); break;
	case (FROM_BELOW):
					tf_pose_in_baselink.setRotation(tf::Quaternion( HALF_SQRT_TWO , 0, HALF_SQRT_TWO , 0)); break;
	case (FROM_RIGHT_SIDEWAYS):
					tf_pose_in_baselink.setRotation(tf::Quaternion( 0 , 0, HALF_SQRT_TWO , HALF_SQRT_TWO)); break;
	case (FROM_RIGHT_UPRIGHT):
					tf_pose_in_baselink.setRotation(tf::Quaternion( -0.5 , -0.5, 0.5 , 0.5)); break;
	case (FROM_ABOVE):
					tf_pose_in_baselink.setRotation(tf::Quaternion( HALF_SQRT_TWO , 0, -HALF_SQRT_TWO , 0)); break;
	case (FROM_LEFT_SIDEWAYS):
					tf_pose_in_baselink.setRotation(tf::Quaternion( -HALF_SQRT_TWO , HALF_SQRT_TWO , 0 , 0)); break;
	case (FROM_LEFT_UPRIGHT):
					tf_pose_in_baselink.setRotation(tf::Quaternion( -0.5 , 0.5, -0.5 , 0.5)); break;
	}

	return tf_pose_in_baselink;

}

bool action_manager::moveGripperToPosition(const tf::Vector3& position, std::string frame_id, approach_direction_t approach,  double max_time , bool wait, std::vector<double>* ik_seed_pos, bool right){

	tf::StampedTransform tf_pose_in_baselink_new(gripperToWrist(makePose(position,  frame_id,  approach)));
	return moveWristRollLinktoPose(tf_pose_in_baselink_new, max_time, wait, ik_seed_pos,right);
}




bool action_manager::moveGrippertoPose(const tf::StampedTransform& tf, double max_time, bool wait, std::vector<double>* ik_seed_pos, bool right){


	tf::StampedTransform tf_new(gripperToWrist(tf));
	return moveWristRollLinktoPose(tf_new, max_time, wait, ik_seed_pos,right);

}

bool action_manager::moveGrippertoPoseWithCollisionChecking(const tf::StampedTransform& tf, double max_time, bool wait, std::string planner, bool right){


	tf::StampedTransform tf_new(gripperToWrist(tf));
	return moveWristRollLinktoPoseWithCollisionChecking(tf_new, max_time, wait, planner,right);

}

bool action_manager::moveGrippertoPositionWithCollisionChecking(const tf::Vector3& position, std::string frame_id, approach_direction_t approach,  double max_time, bool wait,std::string planner,bool right){


	tf::StampedTransform tf_pose_in_baselink_new(gripperToWrist(makePose(position,  frame_id,  approach)));
	return moveWristRollLinktoPoseWithCollisionChecking(tf_pose_in_baselink_new, max_time, wait,planner,right);

}

bool action_manager::moveGrippertoPoseWithOrientationConstraints(const tf::StampedTransform& tf, bool keep_roll, bool keep_pitch, bool keep_yaw, double max_time, bool wait, double tolerance,bool right){

	tf::StampedTransform tf_new(gripperToWrist(tf));
	return moveWristRollLinktoPoseWithOrientationConstraints(tf_new, keep_roll, keep_pitch, keep_yaw, max_time, wait,right);

}


bool action_manager::moveWristRollLinktoPoseWithOrientationConstraints(const tf::StampedTransform& tf, bool keep_roll, bool keep_pitch, bool keep_yaw, double max_time, bool wait, double tolerance,bool right){
	geometry_msgs::PoseStamped pose;
	tf::Stamped<tf::Pose> tf_pose(tf,tf.stamp_, tf.frame_id_);

	tf::poseStampedTFToMsg(tf_pose,pose);
	return moveWristRollLinktoPoseWithOrientationConstraints(pose, keep_roll, keep_pitch, keep_yaw, max_time, wait, tolerance,right);
}

bool action_manager::moveWristRollLinktoPoseWithOrientationConstraints(const geometry_msgs::PoseStamped& pose, bool keep_roll, bool keep_pitch, bool keep_yaw, double max_time, bool wait, double tolerance, bool right){

	if(right){
		if (!arm_r_client_){
			ROS_ERROR("action_manager::pr2_action_interface: collision checking arm server has not been started");
			return false;
		}
	}
	else{
		if (!arm_l_client_){
			ROS_ERROR("action_manager::pr2_action_interface: collision checking arm server has not been started");
			return false;
		}
	}

	bool finished_before_timeout = true;
	arm_navigation_msgs::MoveArmGoal goalA;

	if(right)
		group_name_= "right_arm";
	else
		group_name_= "left_arm";

	goalA.motion_plan_request.group_name = group_name_;
	goalA.motion_plan_request.num_planning_attempts = 1;
	goalA.motion_plan_request.planner_id = std::string("");
	goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
	goalA.motion_plan_request.allowed_planning_time = ros::Duration(25.0);

	goalA.motion_plan_request.goal_constraints.position_constraints.resize(1);
	goalA.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = pose.header.stamp;
	goalA.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id =pose.header.frame_id ;


	if(right)
		goalA.motion_plan_request.goal_constraints.position_constraints[0].link_name = "r_wrist_roll_link";
	else
		goalA.motion_plan_request.goal_constraints.position_constraints[0].link_name = "l_wrist_roll_link";

	goalA.motion_plan_request.goal_constraints.position_constraints[0].position.x = pose.pose.position.x;
	goalA.motion_plan_request.goal_constraints.position_constraints[0].position.y = pose.pose.position.y;
	goalA.motion_plan_request.goal_constraints.position_constraints[0].position.z = pose.pose.position.z;

	goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.type = arm_navigation_msgs::Shape::BOX;
	goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);
	goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);
	goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);

	goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_orientation.w = 1.0;
	goalA.motion_plan_request.goal_constraints.position_constraints[0].weight = 1.0;


	goalA.motion_plan_request.goal_constraints.position_constraints.resize(1);
	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].header.stamp = pose.header.stamp;
	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id = pose.header.frame_id;
	if(right)
		goalA.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = "r_wrist_roll_link";
	else
		goalA.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = "l_wrist_roll_link";

	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x = pose.pose.orientation.x;
	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y = pose.pose.orientation.y;
	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z = pose.pose.orientation.z;
	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w = pose.pose.orientation.w;

	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.04;
	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_pitch_tolerance = 0.04;
	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_yaw_tolerance = 0.04;

	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.04;
	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_pitch_tolerance = 0.04;
	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_yaw_tolerance = 0.04;

	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].weight = 1.0;


	goalA.motion_plan_request.path_constraints.orientation_constraints.resize(1);
	goalA.motion_plan_request.path_constraints.orientation_constraints[0].header.frame_id = pose.header.frame_id ;
	goalA.motion_plan_request.path_constraints.orientation_constraints[0].header.stamp = pose.header.stamp;

	if(right)
		goalA.motion_plan_request.path_constraints.orientation_constraints[0].link_name = "r_wrist_roll_link";
	else
		goalA.motion_plan_request.path_constraints.orientation_constraints[0].link_name = "l_wrist_roll_link";


	goalA.motion_plan_request.path_constraints.orientation_constraints[0].orientation.x = pose.pose.orientation.x;
	goalA.motion_plan_request.path_constraints.orientation_constraints[0].orientation.y = pose.pose.orientation.y;
	goalA.motion_plan_request.path_constraints.orientation_constraints[0].orientation.z = pose.pose.orientation.z;
	goalA.motion_plan_request.path_constraints.orientation_constraints[0].orientation.w = pose.pose.orientation.w;



	goalA.motion_plan_request.path_constraints.orientation_constraints[0].type = arm_navigation_msgs::OrientationConstraint::HEADER_FRAME;
	if (keep_roll)
		goalA.motion_plan_request.path_constraints.orientation_constraints[0].absolute_roll_tolerance = tolerance;
	else
		goalA.motion_plan_request.path_constraints.orientation_constraints[0].absolute_roll_tolerance = M_PI;

	if (keep_pitch)
		goalA.motion_plan_request.path_constraints.orientation_constraints[0].absolute_pitch_tolerance = tolerance;
	else
		goalA.motion_plan_request.path_constraints.orientation_constraints[0].absolute_pitch_tolerance = M_PI;

	if (keep_yaw)
		goalA.motion_plan_request.path_constraints.orientation_constraints[0].absolute_yaw_tolerance = tolerance;
	else
		goalA.motion_plan_request.path_constraints.orientation_constraints[0].absolute_yaw_tolerance = M_PI;



	if(right){
		arm_r_client_->sendGoal(goalA);
		if(wait){
			finished_before_timeout = arm_r_client_->waitForResult(ros::Duration(60.0));
			if (!finished_before_timeout){
				ROS_INFO("action_manager::pr2_action_interface: Action did not finish before time out");
				return false;
			}
			else
			{
				actionlib::SimpleClientGoalState state = arm_r_client_->getState();
				ROS_INFO("action_manager::pr2_action_interface: Action finished: %s",state.toString().c_str());
			}
		}
		if( arm_r_client_->getState() == actionlib::SimpleClientGoalState::ABORTED){
			return false;
		}else{
			return true;
		}
	}
	else{

		arm_l_client_->sendGoal(goalA);

		if(wait){
			finished_before_timeout = arm_l_client_->waitForResult(ros::Duration(60.0));
			if (!finished_before_timeout){
				ROS_INFO("action_manager::pr2_action_interface: Action did not finish before time out");
				return false;
			}
			else
			{
				actionlib::SimpleClientGoalState state = arm_l_client_->getState();
				ROS_INFO("action_manager::pr2_action_interface: Action finished: %s",state.toString().c_str());
			}
		}
		if( arm_l_client_->getState() == actionlib::SimpleClientGoalState::ABORTED){
			return false;
		}else{
			return true;
		}
	}

}

double action_manager::distanceFromFrame(const geometry_msgs::PoseStamped& pose, std::string frame_id){


	// First finding nearest Arm
	tf::TransformListener listener;
	tf::StampedTransform transform;
        ROS_INFO("Transform frame is %s",frame_id.c_str());
        if(frame_id.compare("/base_link") != 0){
	listener.waitForTransform("/base_link",frame_id,
			ros::Time(), ros::Duration(10.0));

	listener.lookupTransform("/base_link",frame_id,
			ros::Time(), transform);}

	tf::Pose tf_push_pose;
	tf::poseMsgToTF(pose.pose,tf_push_pose);

	return transform.getOrigin().distance(tf_push_pose.getOrigin());
}


bool action_manager::rightCloser(const geometry_msgs::PoseStamped& push_pose){

	double distance_r = distanceFromFrame(push_pose,"r_wrist_roll_link");
	double distance_l = distanceFromFrame(push_pose,"l_wrist_roll_link");

	if(distance_l > distance_r)
		return true;
	else
		return false;
}


bool action_manager::graspPlaceAction(const geometry_msgs::PoseStamped& push_pose,const geometry_msgs::PoseStamped& place_pose){

	bool right = rightCloser(push_pose);

	// This assuming input is in the base frame
	ROS_INFO("action_manager::pr2_action_interface: Converting pose to TF");
	tf::Pose push_tf;
	tf::poseMsgToTF(push_pose.pose,push_tf);
	ROS_INFO("action_manager::pr2_action_interface: Converting pose complete");

	// Move to a position that is 10cm above the grasp point
	push_tf.getOrigin().setZ(push_tf.getOrigin().getZ() + 0.10);

	bool success = moveToSide(right);

	success = moveGrippertoPositionWithCollisionChecking(push_tf.getOrigin(),frame_id_,FROM_ABOVE,5.0,true,"ompl",right);

	std::vector<double>* ik_seed_pos;

	if(success)
	{
		ROS_INFO("action_manager::pr2_action_interface: Moved to pre-grasp position, Now open grippers");
		success = controlGripper(right,0); //Open Gripper
		//TODO: Check if this pipeline works
		// first provide new position
		push_tf.getOrigin().setZ(push_tf.getOrigin().getZ() - 0.12);
		success = moveGripperToPosition(push_tf.getOrigin(),frame_id_,FROM_ABOVE,5.0,true,ik_seed_pos,right);
		success = controlGripper(right,1); // Close Gripper
		if(success)
			ROS_INFO("action_manager::pr2_action_interface: Grasp Successful");
		// Now lift the gripper
		push_tf.getOrigin().setZ(push_tf.getOrigin().getZ() + 0.15);
		success = moveGripperToPosition(push_tf.getOrigin(),frame_id_,FROM_ABOVE,5.0,true,ik_seed_pos,right);

		// Now PLACE action
		tf::poseMsgToTF(place_pose.pose,push_tf);
		success = moveGrippertoPositionWithCollisionChecking(push_tf.getOrigin(),frame_id_,FROM_ABOVE,5.0,true,"ompl",right);
		success = controlGripper(right,0); //Open Gripper
		success = moveToSide(right); //Open Gripper
		return true;
	}
	else{
		ROS_INFO("action_manager::pr2_action_interface: Grasp failed");
		return false;
	}
}

bool action_manager::pushAction(const geometry_msgs::PoseStamped& pose, approach_direction_t approach){

	bool right = rightCloser(pose);

	// This assuming input is in the base frame
	ROS_INFO("action_manager::pr2_action_interface: Converting pose to TF");
	tf::Pose push_tf;
	tf::poseMsgToTF(pose.pose,push_tf);
	ROS_INFO("action_manager::pr2_action_interface: Converting pose complete");
	bool success = moveToSide(right);
	// Move to a position that is 5cm relatively in front of the manipulation position
	switch (approach){

	case (FRONTAL):
				push_tf.getOrigin().setZ(push_tf.getOrigin().getX() - 0.050); break;
	case (FROM_RIGHT_SIDEWAYS):
				push_tf.getOrigin().setZ(push_tf.getOrigin().getY() - 0.050); break;
	case (FROM_RIGHT_UPRIGHT):
				push_tf.getOrigin().setZ(push_tf.getOrigin().getY() - 0.050); break;
	case (FROM_LEFT_SIDEWAYS):
				push_tf.getOrigin().setZ(push_tf.getOrigin().getY() + 0.050); break;
	case (FROM_LEFT_UPRIGHT):
				push_tf.getOrigin().setZ(push_tf.getOrigin().getY() + 0.050); break;
	default:
		ROS_INFO("action_manager::pr2_action_interface: Undefined approach direction"); return false;
	}

	success = moveGrippertoPositionWithCollisionChecking(push_tf.getOrigin(),frame_id_,approach,5.0,true,"ompl",right);

	std::vector<double>* ik_seed_pos;

	if(success)
	{
		ROS_INFO("action_manager::pr2_action_interface: Moved to pre-grasp position, Now open grippers");
		success = controlGripper(right,1); //Close Gripper
		// Move to a position that is 5cm relatively in front of the manipulation position
		switch (approach){

		case (FRONTAL):
					push_tf.getOrigin().setZ(push_tf.getOrigin().getX() + 0.150); break;
		case (FROM_RIGHT_SIDEWAYS):
					push_tf.getOrigin().setZ(push_tf.getOrigin().getY() + 0.150); break;
		case (FROM_RIGHT_UPRIGHT):
					push_tf.getOrigin().setZ(push_tf.getOrigin().getY() + 0.150); break;
		case (FROM_LEFT_SIDEWAYS):
					push_tf.getOrigin().setZ(push_tf.getOrigin().getY() - 0.150); break;
		case (FROM_LEFT_UPRIGHT):
					push_tf.getOrigin().setZ(push_tf.getOrigin().getY() - 0.150); break;
		default:
			ROS_INFO("action_manager::pr2_action_interface: Undefined approach direction"); return false;
		}
		//TODO: Check if this pipeline works
		// first provide new position
		success = moveGripperToPosition(push_tf.getOrigin(),frame_id_,approach,5.0,true,ik_seed_pos,right);
		if(success)
			ROS_INFO("action_manager::pr2_action_interface: Push Successful");
		// Now lift the gripper
		push_tf.getOrigin().setZ(push_tf.getOrigin().getZ() + 0.15);
		success = moveGripperToPosition(push_tf.getOrigin(),frame_id_,approach,5.0,true,ik_seed_pos,right);
		success = moveToSide(right);
		return true;
	}
	else{
		ROS_INFO("action_manager::pr2_action_interface: Grasp failed");
		return false;
	}
}

} // end of namespace

int main(int argc, char **argv){

	ros::init (argc, argv, "/pr2_action_interface");
	ros::NodeHandle nh("~");
	std::string action_name = "/pr2_action_interface";
	action_manager_pr2::action_manager aa(nh,action_name);
	ros::spin();
}



