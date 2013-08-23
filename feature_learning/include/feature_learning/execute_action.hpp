
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
 * @b header file for the executing various actions while testing the feature selection pipeline
 */

#ifndef EXECUTE_ACTION_HPP
#define EXECUTE_ACTION_HPP

#include <math.h>
#include "feature_learning/extract_features.hpp"
#include "feature_learning/ExtractFeatures.h"

// Arm trajectory client and controller stuff
#include <arm_controller_interface/joint_trajectory_client.h>
#include <arm_controller_interface/switch_controller_stack_client.h>
#include <arm_manipulation_tools/arm_interface.h>
#include <usc_utilities/rviz_marker_manager.h>
#include <arm_motion_primitives/poke.h>
#include <arm_motion_primitives/grasp.h>
#include <arm_world_state/world_state.h>

//Action server includes
#include <actionlib/server/simple_action_server.h>
#include "feature_learning/ExecuteActionAction.h"

//Tabletop Server includes
#include <tabletop_segmenter/TabletopSegmentation.h>

using namespace arm_manipulation_tools;
using namespace arm_motion_primitives;
using namespace arm_world_state;

namespace feature_learning {

class execute_action{

public:

	ArmInterface manipulation_object_l_,manipulation_object_r_;

	Poke poke_object_;

	WorldState world_state_;

	arm_controller_interface::HeadJointTrajectoryClient head_joint_trajectory_client_;
    arm_controller_interface::SwitchControllerStackClient switch_controller_stack_;

protected:

	ros::NodeHandle nh_;
	tf::TransformListener listener_;

private:

	// Private NodeHandle
	ros::NodeHandle nh_priv_;

	// Action Server related objects
	actionlib::SimpleActionServer<feature_learning::ExecuteActionAction> action_server_;
	std::string action_name_;
	feature_learning::ExecuteActionResult result_;

	//Tabletop Segmenter Service Client
	ros::ServiceClient tabletop_client_,extract_feature_client_;

	//Get a string representation for the name of the class
	std::string getName () const {return ("execute_action");}

public:

	execute_action(ros::NodeHandle& nh);

	~execute_action();

	void executeCallBack(const feature_learning::ExecuteActionGoalConstPtr &goal);

	geometry_msgs::PoseStamped getSurfacePose();

	geometry_msgs::PoseStamped getGripperPose();

	geometry_msgs::PoseStamped getViewpointPose();

	bool getTransform(tf::StampedTransform& transform, const string& from, const string& to) const;

	bool doActionOnPoint(const geometry_msgs::Point &action_point, int action, ArmInterface& manipulation_object);

	void getPushPose(const geometry_msgs::PoseStamped &start_pose, geometry_msgs::PoseStamped &end_pose, double direction);

	void getGraspPose(const geometry_msgs::PoseStamped &start_pose, geometry_msgs::PoseStamped &end_pose, double direction);

	bool graspNode(geometry_msgs::PoseStamped grasp_pose, ArmInterface& manipulation_object);

	bool pushNode(geometry_msgs::PoseStamped push_pose, double y_direction, ArmInterface& manipulation_object);

};

}

#endif
