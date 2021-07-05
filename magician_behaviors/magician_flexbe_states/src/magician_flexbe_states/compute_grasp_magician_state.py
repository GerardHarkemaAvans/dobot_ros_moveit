#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Delft University of Technology
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Delft University of Technology nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Authors: the HRWROS mooc instructors

import rospy
import sensor_msgs

import numpy as np

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

import math
import sys
import copy
import moveit_commander
import moveit_msgs.msg

from tf.transformations import *

import geometry_msgs
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs
import actionlib

'''
Created on Sep 5 2018

@author: HRWROS mooc instructors
Adapted to Magician by: Gerard Harkema

This state provides the joint configuration to grasp the box in the factory simulation of the MOOC "Hello (Real) World with ROS", given the pose of the box as provided by the DetectPartCameraState
'''

class ComputeMagicianGraspState(EventState):
    '''
    Computes the joint configuration needed to grasp the part given its pose.

    -- joint_names  string[]    Names of the joints
    ># offset            float        Some offset
    ># rotation          float        Rotation?
    ># group   string        Name of the group for which to compute the joint values for grasping.
  ># namespace    string          Name of the prefix of the move group to be used for planning.
    ># tool_link        string        e.g. "ee_link"
    ># pose                PoseStamped    pose of the part to pick
    #> joint_values float[]        joint values for grasping
    #> joint_names    string[]    names of the joints

    <= continue       if a grasp configuration has been computed for the pose
    <= failed           otherwise.
    '''

    def __init__(self, joint_names):
      # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
      super(ComputeMagicianGraspState, self).__init__(outcomes = ['continue', 'failed'], input_keys = ['group', 'namespace', 'tool_link','pose', 'offset', 'rotation'], output_keys = ['joint_values','joint_names'])
      
      self._joint_names = joint_names
      self._failed = True
       
      # tf to transform the object pose
      self._tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0)) #tf buffer length
      self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
    def execute(self, userdata):
      # This method is called periodically while the state is active.
      # Main purpose is to check state conditions and trigger a corresponding outcome.
      # If no outcome is returned, the state will stay active.

      #rospy.logwarn(userdata.pose)

      if self._failed == True:
        return 'failed'

      if int(self._srv_result.error_code.val) == 1:
        sol_js = self._srv_result.solution.joint_state

        joint_names = self._joint_names

        jname_idx = [sol_js.name.index(jname) for jname in joint_names]
        j_angles = map(sol_js.position.__getitem__, jname_idx)
        # solution_dict = dict(zip(joint_names, j_angles))
        userdata.joint_values = copy.deepcopy(j_angles)
        userdata.joint_names = copy.deepcopy(joint_names)
        Logger.loginfo("joint_names: %s" % userdata.joint_names)
        Logger.loginfo("joint_values: %s" % userdata.joint_values)
        return 'continue'
      else:
        Logger.logerr("error: %s" % self._srv_result)
        Logger.loginfo("ComputeGraspState::Execute state - failed.  Returned: %d", int(self._srv_result.error_code.val) )
        return 'failed'

    def on_enter(self, userdata):
      # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
      # It is primarily used to start actions which are associated with this state.

  
      # Get transform between camera and robot1_base
      while True:
          rospy.sleep(0.1)
          try:
            Logger.loginfo(userdata.pose)                
            #target_pose = self._tf_buffer.transform(userdata.pose, 'world')
            target_transform = self._tf_buffer.lookup_transform('world', userdata.pose, rospy.Time())
            break
          except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
              Logger.logerr("ComputeGraspState::on_enter - Failed to transform to world")
              continue

      Logger.loginfo("target transform : %s" % target_transform)

      target_pose = PoseStamped()
      target_pose.header = target_transform.header
      target_pose.pose.position.x = target_transform.transform.translation.x
      target_pose.pose.position.y = target_transform.transform.translation.y
      target_pose.pose.position.z = target_transform.transform.translation.z
      target_pose.pose.orientation.x = target_transform.transform.rotation.x
      target_pose.pose.orientation.y = target_transform.transform.rotation.y      
      target_pose.pose.orientation.z = target_transform.transform.rotation.z      
      target_pose.pose.orientation.w = target_transform.transform.rotation.w      

      
      Logger.loginfo("target pose : %s" % target_pose)

      self._srv_name = userdata.namespace + '/compute_ik'
      self._ik_srv = ProxyServiceCaller({self._srv_name: GetPositionIK})        
      
      target_pose.pose.position.z += userdata.offset + 0.0
      
      
      # use ik service to compute joint_values
      self._srv_req = GetPositionIKRequest()
      #Logger.logerr(self._group)
      #Logger.logerr(self._namespace + '/joint_states')
      self._srv_req.ik_request.group_name = userdata.group 
      self._srv_req.ik_request.robot_state.joint_state = rospy.wait_for_message(userdata.namespace + '/joint_states', sensor_msgs.msg.JointState)

      Logger.logerr(userdata.tool_link)
      self._srv_req.ik_request.ik_link_name = userdata.tool_link  # TODO: this needs to be a parameter
      self._srv_req.ik_request.pose_stamped = target_pose
      self._srv_req.ik_request.avoid_collisions = False
      self._srv_req.ik_request.attempts = 500
      try:
          Logger.logerr("calling ik")
          self._srv_result = self._ik_srv.call(self._srv_name, self._srv_req)
          self._failed = False

      except Exception as e:
          Logger.logerr('Could not call IK: %s' % str(e))
          self._failed = True                  


    def on_exit(self, userdata):
        # This method is called when an outcome is returned and another state gets active.
        # It can be used to stop possibly running processes started by on_enter.
        pass # Nothing to do

    def on_start(self):
        # This method is called when the behavior is started.
        # If possible, it is generally better to initialize used resources in the constructor
        # because if anything failed, the behavior would not even be started.
        pass

    def on_stop(self):
        # This method is called whenever the behavior stops execution, also if it is cancelled.
        # Use this event to clean up things like claimed resources.
        pass # Nothing to do

