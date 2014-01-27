#!/usr/bin/env python
#
# Copyright (c) 2011, Bosch LLC
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
# Authors: Christian Bersch Bosch LLC

import roslib
roslib.load_manifest("simple_robot_control")
import rospy
import math
import numpy

import threading

import geometry_msgs.msg
import tf.listener
import tf.transformations



class Base:
    def __init__(self):
        
        self.base_vel_cmd_pub = rospy.Publisher("/base_controller/command", geometry_msgs.msg.Twist)
        self.listener = tf.TransformListener()
        self.lock = threading.Lock()
#        self.thread = None
        
    
    def driveForward(self,distance,speed = 0.1, wait = True):
        """
        if distance is positive it drives forward else backwards
        """
        speed = abs(speed)       
        velocity = geometry_msgs.msg.Twist()
        if distance < 0.0:
            speed = -1.0 * speed
            distance = -1.0 * distance
        velocity.linear.x = speed
        self.driveTwist(distance, velocity, wait)
        
    def driveLeft(self,distance,speed = 0.1, wait = True):
        """
        if distance is positive it drives left else right
        """
        speed = abs(speed)       
        velocity = geometry_msgs.msg.Twist()
        if distance < 0.0:
            speed = -1.0 * speed
            distance = -1.0 * distance
        velocity = geometry_msgs.msg.Twist()
        velocity.linear.y = speed
        self.driveTwist(distance, velocity, wait)
    
    
    def _driveTwist(self, distance, velocity):
        velocity.angular.x = velocity.angular.y = velocity.angular.z = velocity.linear.z = 0
        #get current pose
        now = rospy.Time.now()
        self.listener.waitForTransform('base_footprint', 'odom_combined', now, rospy.Duration(5.0))
        self.lock.acquire()
        (trans_start,rot_start) = (trans_end,rot_end) = self.listener.lookupTransform( 'base_footprint', 'odom_combined', rospy.Time(0))
        #keep moving until distance travelled
        while numpy.linalg.norm(numpy.array(trans_start) - trans_end) < distance and not rospy.is_shutdown():
            self.base_vel_cmd_pub.publish(velocity)
            rospy.sleep(0.05)
            (trans_end,rot_end) = self.listener.lookupTransform( 'base_footprint', 'odom_combined', rospy.Time(0))
        self.lock.release()
        
    def driveTwist(self, distance, velocity, wait):
        """
        executes translational drive command as specified in velocity twist
        """
        if not isinstance(velocity,  geometry_msgs.msg.Twist):
            raise TypeError('velocity must be of type geometry_msgs.msg.Twist')
        
        if wait:
            self._driveTwist(distance, velocity)
        else:
            thread = threading.Thread(target=self._driveTwist, args = (distance, velocity))
            thread.start()
        
        
        
    def _turnLeft(self,radians, speed = 0.2):
        speed = abs(speed)
        if radians < 0.0:
            speed = -1.0 * speed
            radians = -1.0 * radians
        radians = radians % math.pi
        velocity = geometry_msgs.msg.Twist()
        velocity.angular.z = speed      
        #get current pose
        now = rospy.Time.now()  
        self.listener.waitForTransform('base_footprint', 'odom_combined', now, rospy.Duration(5.0))
        self.lock.acquire()
        (trans_start,rot_start) = (trans_end,rot_end) = self.listener.lookupTransform( 'base_footprint', 'odom_combined', rospy.Time(0))
        #keep moving until rotation reached - since robot only rotates around z axis one can compare the w component of the quaternions to half the angle
        angle_turned = 0.0
        rot_start_inv = tf.transformations.quaternion_inverse(rot_start)
        while  angle_turned < radians:
            
            self.base_vel_cmd_pub.publish(velocity)
            rospy.sleep(0.05)
            (trans_end,rot_end) = self.listener.lookupTransform( 'base_footprint', 'odom_combined', rospy.Time(0))
            euler_angles = tf.transformations.euler_from_quaternion(tf.transformations.quaternion_multiply(rot_start_inv, rot_end))
            angle_turned = abs( euler_angles[2])
#            print "angle_turned", angle_turned
        self.lock.release()
            
            
    def turnLeft(self,radians, speed = 0.2, wait = True):
        """
        rotates the base by radians with speed. positive radians value will cause counterclockwise turning, negative radians will cause clockwise turning
        """
        if wait:
            self._turnLeft(radians, speed)
        else:
            thread = threading.Thread(target=self._turnLeft, args = (radians, speed))
            thread.start()
        
        
    def isDone(self):
        return not self.lock.locked()
            
               
        

        
    

