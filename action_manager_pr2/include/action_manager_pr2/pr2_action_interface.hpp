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
 */

#ifndef PR2_ACTION_INTERFACE_HPP
#define PR2_ACTION_INTERFACE_HPP

#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <tf/transform_datatypes.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <tf/transform_listener.h>
#include "action_manager_pr2/ControllerAction.h"
#include "action_manager_msgs/Controller.h"

// USC Utilities for markers
#include <usc_utilities/rviz_marker_manager.h>

#define HALF_SQRT_TWO 0.707106781


namespace action_manager_pr2{

class action_manager{

public:

	// Clients for PR2 control
	typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;
	typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
	typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> MoveArmClient;
	typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajectoryClient;
	typedef actionlib::SimpleActionServer<action_manager_pr2::ControllerAction> ActionServer;
	typedef ActionServer::GoalHandle GoalHandle;

private:

	ros::NodeHandle nh_priv_;

	PointHeadClient *point_head_client_;
	GripperClient *gripper_r_client_, *gripper_l_client_;
	MoveArmClient *arm_r_client_, *arm_l_client_;
	TrajectoryClient *r_traj_client_, *l_traj_client_;

	const static double wrist_speed_ = 2.0; //2 seconds per revolution
	const static double gripper_length_ = 0.18;

protected:

	ros::NodeHandle nh_;

	ros::Publisher pose_publisher_;

	//Declaring action server and related variables
	ActionServer as_;
	action_manager_pr2::ControllerFeedback feedback_;
	action_manager_pr2::ControllerResult result_;
	std::string action_name_;

	kinematics_msgs::GetPositionIK ik_service_client_;

	usc_utilities::RvizMarkerManager marker_;

	// Pr2 Controller action servers to call
	std::string gripper_r_srv_, gripper_l_srv_, head_srv_,arm_r_srv_, arm_l_srv_,r_joint_srv_,l_joint_srv_;

	// Pr2 Controller services to listen to
	std::string r_ik_service_name_, l_ik_service_name_,joint_states_service_;


	// topics and frames
	std::string controller_topic_, frame_id_;

	//frames_ids of interest
	std::string head_point_frame_, group_name_;

	//global identifier for joint names
	std::vector<std::string> joint_names_;

	// Vector of joint angles
	std::vector<double> current_joint_angles_;



public:

	action_manager(ros::NodeHandle &nh, const std::string action_name);

	~action_manager();

	void execute(const action_manager_pr2::ControllerGoalConstPtr& goal);

	bool controlGripper(int hand = 1, int goal = 0);
	// goal 0 for open and 1 for close, hand 1 for right and 0 for left

	bool controlHead(const std::string& pointing_frame_id, const geometry_msgs::PointStamped& target_point, int action = 0); // 0 for point 1 to track

	bool controlArm(const geometry_msgs::PoseStamped& start_pose, const geometry_msgs::PoseStamped& end_pose, const std::string& frame_id, int arm = 1, int action = 0, int direction = 0);

	std::vector<std::string> getJointNames(bool right = true);

	bool updateJointStates(bool right = true);

	/* ****************************
 	    Joint Angle control
	   **************************** */
	// moves arm to joint angle position. positions should contain a multiple of the 7 arm joint positions to create a trajectory
	bool goToJointPos (const std::vector<double>& positions , double max_time = 3.0, bool wait = true, bool right = true);

	// moves arm to joint position. positions should contain a multiple of the 7 arm joint positions to create a trajectory
	bool goToJointPos (const double* positions , int num_positions, double max_time = 3.0, bool wait = true, bool right = true);

	// uses ompl path planner and moves arm to  joint position. positions should contain a multiple of the 7 arm joint positions to create a trajectory.
	bool goToJointPosWithCollisionChecking (const std::vector<double>& positions, double max_time = 3.0, bool wait = true, bool right = true);

	// Rotates wrist by angle in radians. speed is the amount of time that it would take for one revolution.
	bool rotateWrist(double radians, double wrist_speed = wrist_speed_ ,bool wait = true, bool right = true );

	// get joint angles
	std::vector<double> getCurrentJointAngles(bool right = true);

	/* ****************************
 	    Euclidian position control
	   **************************** */

	// get IK from kinematics server
	bool getIK(const geometry_msgs::PoseStamped& pose,  std::vector<double>& joint_angles, std::vector<double>* ik_seed_pos = NULL, bool right = true);

	enum approach_direction_t { FRONTAL, FROM_BELOW, FROM_ABOVE, FROM_RIGHT_SIDEWAYS, FROM_RIGHT_UPRIGHT, FROM_LEFT_UPRIGHT, FROM_LEFT_SIDEWAYS};

	// gets IK and moves gripper_tool_frame to pose specified in postion and the orientation of the approach direction relative to base coordinate frame
	bool moveGripperToPosition(const tf::Vector3& position,  std::string frame_id, approach_direction_t approach = FRONTAL,  double max_time =5.0, bool wait=true, std::vector<double>* ik_seed_pos = 0, bool right = true);

	// gets IK and moves gripper_tool_frame to pose specified in postion and the orientation of the approach direction relative to base coordinate frame
	bool moveGrippertoPositionWithCollisionChecking(const tf::Vector3& position,  std::string frame_id, approach_direction_t approach = FRONTAL,  double max_time =5.0, bool wait=true, std::string planner = "ompl",  bool right = true);

	// gets IK and moves gripper_tool_frame to pose specified in pose. The orientation can be overridden by above enums.
	bool moveGrippertoPose(const tf::StampedTransform& pose,   double max_time =5.0, bool wait=true, std::vector<double>* ik_seed_pos = 0, bool right = true);

	// uses ompl path planner and moves gripper_tool_frame to pose specified in pose. The orientation can be overridden by above enums.
	bool moveGrippertoPoseWithCollisionChecking(const tf::StampedTransform& pose,  double max_time =5.0, bool wait=true, std::string planner = "ompl", bool right = true);

	// uses ompl planner to calculate path and moves wrist roll link to pose specified and keeps the set orientations on the motion path within the tolerance (angle in radians)
	bool moveGrippertoPoseWithOrientationConstraints(const tf::StampedTransform& tf, bool keep_roll, bool keep_pitch, bool keep_yaw, double max_time =5.0, bool wait=true, double tolerance = 0.2, bool right = true);

	// gets IK and moves wrist roll link to pose specified in tf
	bool moveWristRollLinktoPose(const tf::StampedTransform& pose,  double max_time =5.0, bool wait=true, std::vector<double>* ik_seed_pos = 0, bool right = true);

	// gets IK and moves wrist roll link to pose specified in pose
	bool moveWristRollLinktoPose(const geometry_msgs::PoseStamped& pose,  double max_time =5.0, bool wait=true, std::vector<double>* ik_seed_pos = 0, bool right = true);

	// uses ompl planner to calculate path and moves wrist roll link to pose specified in pose
	bool moveWristRollLinktoPoseWithCollisionChecking(const geometry_msgs::PoseStamped& pose,  double max_time =5.0, bool wait=true, std::string planner = "ompl", bool right = true);

	// uses ompl planner to calculate path and moves wrist roll link to pose specified
	bool moveWristRollLinktoPoseWithCollisionChecking(const tf::StampedTransform& pose,  double max_time =5.0, bool wait=true, std::string planner = "ompl", bool right = true);

	// uses ompl planner to calculate path and moves wrist roll link to pose specified and keeps the set orientations on the motion path within the tolerance (angle in radians)
	bool moveWristRollLinktoPoseWithOrientationConstraints(const tf::StampedTransform& pose, bool keep_roll, bool keep_pitch, bool keep_yaw, double max_time =5.0, bool wait=true, double tolerance = 0.2, bool right = true);

	// uses ompl planner to calculate path and moves wrist roll link to pose specified and keeps the set orientations on the motion path within the tolerance (angle in radians)
	bool moveWristRollLinktoPoseWithOrientationConstraints(const geometry_msgs::PoseStamped& pose, bool keep_roll, bool keep_pitch, bool keep_yaw, double max_time =5.0, bool wait=true, double tolerance = 0.2, bool right = true);

	// Checks whether current arm position are the same as in pos_vec (with some epsilon).
	bool isAtPos(const std::vector<double>& pos_vec, bool right = true);

    // Move arm into tuck position.
	bool tuck(bool right = true);

	// Move arm into stretched out position.
	bool stretch( bool right = true);

	bool moveToSide(bool right = true);

	bool moveToPreGrasp(bool right = true);

	bool goZero();

	bool moveArmToPoseWithCart(const geometry_msgs::PoseStamped& pose, bool right = true);

	// Various Behaviors that are executed by the arm thats nearest to the base_link pose

	// Returns distance of current point from link in base frame
	double distanceFromFrame(const geometry_msgs::PoseStamped& pose, std::string frame_id);
	// Checks closest hand to point of manipulation, if right is close returns true
	bool rightCloser(const geometry_msgs::PoseStamped& pose);

	bool pushAction(const geometry_msgs::PoseStamped& pose, approach_direction_t approach = FRONTAL);

	bool graspPlaceAction(const geometry_msgs::PoseStamped& push_pose,const geometry_msgs::PoseStamped& place_pose);


private:

	  tf::StampedTransform gripperToWrist(const tf::StampedTransform& pose);
	  tf::StampedTransform wristToGripper(const tf::StampedTransform& pose);
	  tf::StampedTransform makePose(const tf::Vector3& position, std::string frame_id, approach_direction_t approach);

};
}

#endif
