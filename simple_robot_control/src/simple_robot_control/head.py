#!/usr/bin/env python
#
# Copyright (c) 2010, Bosch LLC
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Bosch LLC nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Christian Bersch, Bosch LLC

import roslib
roslib.load_manifest("simple_robot_control")
import rospy

from pr2_controllers_msgs.msg import *
import actionlib

class Head:
    def __init__(self):
        
        self.headClient = actionlib.SimpleActionClient("head_traj_controller/point_head_action", pr2_controllers_msgs.msg.PointHeadAction)
        
        if not self.headClient.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Could not connect to head_traj_controller/point_head_action")
            exit(1)
        
    def lookAtPoint(self, x , y, z, frame_id, speed = 1.0):
        goal = pr2_controllers_msgs.msg.PointHeadGoal()
        goal.pointing_frame = "high_def_frame"
        goal.pointing_axis.x = 1.0
        goal.pointing_axis.y = 0.0
        goal.pointing_axis.z = 0.0
        goal.target.header.frame_id = frame_id
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(0.5)
        goal.max_velocity = speed
        self.headClient.send_goal_and_wait(goal)
    

