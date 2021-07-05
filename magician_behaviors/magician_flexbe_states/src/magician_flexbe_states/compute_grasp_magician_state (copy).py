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

	-- joint_names		string[]	Names of the joints
	># offset		float		Some offset
	># rotation		float		Rotation?
	># group       	string		Name of the group for which to compute the joint values for grasping.
        ># namespace    string          Name of the prefix of the move group to be used for planning.
	># tool_link		string		e.g. "ee_link"
	># pose			PoseStamped	pose of the part to pick
	#> joint_values		float[]		joint values for grasping
	#> joint_names		string[]	names of the joints

	<= continue 				if a grasp configuration has been computed for the pose
	<= failed 				otherwise.
	'''

	def __init__(self, joint_names):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(ComputeMagicianGraspState, self).__init__(outcomes = ['continue', 'failed'], input_keys = ['group', 'namespace', 'tool_link','pose', 'offset', 'rotation'], output_keys = ['joint_values','joint_names'])

		self._joint_names = joint_names

		self._failed = True
		# tf to transfor the object pose
		self._tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0)) #tf buffer length
		self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)


	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.

		#rospy.logwarn(userdata.pose)

		if self._failed == True:
			return 'failed'
			
    if 0 :
		  if int(self._srv_result.error_code.val) == 1:
			  sol_js = self._srv_result.solution.joint_state

			  joint_names = self._joint_names

			  jname_idx = [sol_js.name.index(jname) for jname in joint_names]
			  j_angles = map(sol_js.position.__getitem__, jname_idx)
			  # solution_dict = dict(zip(joint_names, j_angles))
			  userdata.joint_values = copy.deepcopy(j_angles)
			  userdata.joint_names = copy.deepcopy(joint_names)
			  #Logger.logerr(userdata.joint_names)
			  #Logger.logerr(userdata.joint_values)
			  return 'continue'
		  else:
			  Logger.logerr(self._srv_result)
			  Logger.loginfo("ComputeGraspState::Execute state - failed.  Returned: %d", int(self._srv_result.error_code.val) )
			  return 'failed'


	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.

		self._group = userdata.group
		self._namespace = userdata.namespace
		self._tool_link = userdata.tool_link

		self._offset = userdata.offset
		self._rotation = userdata.rotation

		self._srv_name = userdata.namespace + '/compute_ik'
		self._ik_srv = ProxyServiceCaller({self._srv_name: GetPositionIK})

    if 0 :
		  self._robot1_client = actionlib.SimpleActionClient(userdata.namespace + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
		  server_up = self._robot1_client.wait_for_server(rospy.Duration(10.0))
		  if not server_up:
			  Logger.loginfo('Execute Trajectory server is not available for robot')
			  self._failed = True
			  return
		  Logger.loginfo('Execute Trajectory server is available for robot!!!!!!!!!!')

    
		# Get transform between camera and robot1_base
		while True:
			rospy.sleep(0.1)
			#try:
			Logger.loginfo(userdata.pose)				
			target_pose = self._tf_buffer.transform(userdata.pose, 'world')
			#	break
			#except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			#	Logger.logerr("ComputeGraspState::on_enter - Failed to transform to world")
			#	continue

		Logger.loginfo(target_pose)
		if 0:
		  # the grasp pose is defined as being located on top of the item
		  target_pose.pose.position.z += self._offset + 0.0
		  Logger.loginfo('Ha ha')

		  # rotate the object pose 180 degrees around - now works with -90???
		  q_orig = [target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w]


		  test_flag = True
			  
		  if not test_flag:
			  q_orig = [0, 0, 0, 1] # deze moet uiteindelijk weg
			  #q_rot = quaternion_from_euler(self._rotation, math.pi/2.0, 0) # math.pi/2.0 added by gerard en werkt !!
			  #q_rot = quaternion_from_euler(math.pi/2.0, 0, math.pi/-2.0)
			  q_rot = quaternion_from_euler(0, math.pi/2.0, math.pi/2.0)
			  #q_rot = quaternion_from_euler(0, 0, 0)
			  res_q = quaternion_multiply(q_rot, q_orig)

		  else:
			  Logger.logerr('testing')
			  break_flag = False
			  for x in np.arange(-math.pi*2.0, math.pi*2.0, 0.1):
				  #Logger.logerr(x)
				  for y in np.arange(-math.pi*2.0, math.pi*2.0, 0.1):
					  for z in np.arange(-math.pi*2.0, math.pi*2.0, 0.1):
						  #Logger.logerr(y)
						  q_rot = quaternion_from_euler(x, y, z)
						  q_rotb = quaternion_from_euler(x, y, z)
						  res_q = quaternion_multiply(q_rot, q_orig)
						  '''
						  if res_q[0] > -0.05 and res_q[0] < 0.05 and \
							  res_q[1] > 0.705 and res_q[1] < 0.71 and \
							  res_q[2] > -0.05 and res_q[2] < 0.05 and \
							  res_q[3] > 0.705 and res_q[3] < 0.71:
							  Logger.logerr('solution found 1')
							  Logger.logerr(res_q)
							  break_flag = True
							  break
						  '''
						  (roll, pitch, yaw) = euler_from_quaternion(res_q)
						  if pitch > 1.56 and pitch < 1.58 and \
							  yaw > -0.05 and yaw < 0.05 and \
							  roll > -0.05 and roll < 0.05:
							  Logger.logerr('solution found')							
							  Logger.logerr(pitch)
							  Logger.logerr(jaw)
							  Logger.logerr(roll)
							  break_flag = True
							  break
					  if break_flag:
						  break
				  if break_flag:
					  break
					  
		  if 1 :		

			  Logger.logerr('q_orig')
			  Logger.logerr(q_orig)
			  Logger.logerr(euler_from_quaternion(q_orig))

			  Logger.logerr('q_rot')
			  Logger.logerr(q_rot)
			  Logger.logerr(euler_from_quaternion(q_rot))
			  Logger.logerr('res_q')
			  Logger.logerr(res_q)
			  Logger.logerr(euler_from_quaternion(res_q))
			  
			  #Logger.logerr(euler_from_quaternion(q_rotb))
			  #Logger.logerr(x)
			  #Logger.logerr(y)
			  #Logger.logerr(z)
			  
			  Logger.logerr(res_q)
			  Logger.logerr(euler_from_quaternion(res_q))
		  
		  target_pose.pose.orientation = geometry_msgs.msg.Quaternion(*res_q)
				  
		  # use ik service to compute joint_values
		  self._srv_req = GetPositionIKRequest()
		  #Logger.logerr(self._group)
		  #Logger.logerr(self._namespace + '/joint_states')
		  self._srv_req.ik_request.group_name = self._group 
		  self._srv_req.ik_request.robot_state.joint_state = rospy.wait_for_message(self._namespace + '/joint_states', sensor_msgs.msg.JointState)

		  #self._srv_req.ik_request.robot_state.joint_state = rospy.wait_for_message('ariac/joint_states', sensor_msgs.msg.JointState)
		  Logger.logerr(self._tool_link)
		  self._srv_req.ik_request.ik_link_name = self._tool_link  # TODO: this needs to be a parameter
		  self._srv_req.ik_request.pose_stamped = target_pose
		  self._srv_req.ik_request.avoid_collisions = False
		  self._srv_req.ik_request.attempts = 500
		  try:
			  self._srv_result = self._ik_srv.call(self._srv_name, self._srv_req)
			  self._failed = False

		  except Exception as e:
			  Logger.logerr('Could not call IK: ' + str(e))
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
