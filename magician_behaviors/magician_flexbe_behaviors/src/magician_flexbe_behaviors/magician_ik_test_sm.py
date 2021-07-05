#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from magician_flexbe_states.compute_grasp_magician_state import ComputeMagicianGraspState
from magician_flexbe_states.moveit_to_joints_magician_state import MoveitToJointsMagicianState
from magician_flexbe_states.srdf_to_magician_state import SrdfStateToMagician
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 08 2021
@author: Gerard Harkema
'''
class magician_ik_testSM(Behavior):
	'''
	This is the invers kinematics test voor de magician
	'''


	def __init__(self):
		super(magician_ik_testSM, self).__init__()
		self.name = 'magician_ik_test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:826 y:88, x:425 y:303
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.config_name_home = 'home'
		_state_machine.userdata.move_group = 'magician_arm'
		_state_machine.userdata.action_topic_namespace = ''
		_state_machine.userdata.action_topic = '/move_group'
		_state_machine.userdata.robot_name = ''
		_state_machine.userdata.config_name_left = 'left'
		_state_machine.userdata.config_name_right = 'right'
		_state_machine.userdata.pose = "testpoint"
		_state_machine.userdata.namespace = ''
		_state_machine.userdata.tool_link = 'magician_end_link'
		_state_machine.userdata.offset = 0.0
		_state_machine.userdata.rotation = 0.0
		_state_machine.userdata.group = 'magician_arm'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:84 y:74
			OperatableStateMachine.add('MoveHome',
										SrdfStateToMagician(),
										transitions={'reached': 'ComputeGraspState', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name_home', 'move_group': 'move_group', 'action_topic_namespace': 'action_topic_namespace', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:625 y:81
			OperatableStateMachine.add('MovoToGraspPosition',
										MoveitToJointsMagicianState(),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'action_topic_namespace': 'action_topic_namespace', 'move_group': 'move_group', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:426 y:79
			OperatableStateMachine.add('ComputeGraspState',
										ComputeMagicianGraspState(joint_names=["magician_joint1", "magician_joint2", "magician_joint3"]),
										transitions={'continue': 'MovoToGraspPosition', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'group': 'group', 'namespace': 'namespace', 'tool_link': 'tool_link', 'pose': 'pose', 'offset': 'offset', 'rotation': 'rotation', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
