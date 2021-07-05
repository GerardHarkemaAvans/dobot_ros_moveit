#!/usr/bin/env python

import rospy
import math
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, MoveItErrorCodes
from magician_hardware.srv import SetPTPCmd, SetPTPCmdRequest, SetPTPCmdResponse
from magician_hardware.srv import GetQueuedCmdCurrentIndex, GetQueuedCmdCurrentIndexRequest, GetQueuedCmdCurrentIndexResponse


'''
Created on 15.06.2015

@author: Philipp Schillinger
@modified: Gerard Harkema
'''


class MoveitToJointsMagicianState(EventState):
  '''
  Uses MoveIt to plan and move the specified joints to the target configuration.

  ># move_group        string        Name of the move group to be used for planning.

  ># action_topic_namespace    string        Name of the namespace of the move group to be used for planning.
                                  Specified joint names need to exist in the given group.
  ># action_topic     string         Topic on which MoveIt is listening for action calls.

  ># joint_names        string[]    Names of the joints to set.
                                  Does not need to specify all joints.
  ># joint_values        float[]        Target configuration of the joints.
                                  Same order as their corresponding names in joint_names.

  <= reached                         Target joint configuration has been reached.
  <= planning_failed                 Failed to find a plan to the given joint configuration.
  <= control_failed                 Failed to move the arm along the planned trajectory.

    '''


  def __init__(self):
    '''
    Constructor
    '''
    super(MoveitToJointsMagicianState, self).__init__(
        outcomes=['reached', 'planning_failed', 'control_failed'],
        input_keys=['action_topic_namespace', 'move_group', 'action_topic', 'joint_values', 'joint_names'])
    
    self._planning_failed = False
    self._control_failed = False
    self._success = False
      
    self._real_world = True
    self.service_timeout = 0.5    

    Logger.loginfo('Waiting for magician services...')
    try:
      rospy.wait_for_service('/dobot_magician_server/SetPTPCmd', self.service_timeout)
    except rospy.ROSException, e:
      #Logger.logwarn('Magician: SetPTPCmd service not up')
      self._real_world = False
    self.SetPTPCmd_srv = rospy.ServiceProxy('/dobot_magician_server/SetPTPCmd', SetPTPCmd)
    if(self._real_world):
      try:
        rospy.wait_for_service('/dobot_magician_server/GetQueuedCmdCurrentIndex', self.service_timeout)
      except rospy.ROSException, e:
        #Logger.logwarn('Magician: GetQueuedCmdCurrentIndex service not up')
        self._real_world = False
      self.GetQueuedCmdCurrentIndex_srv = rospy.ServiceProxy('/dobot_magician_server/GetQueuedCmdCurrentIndex', GetQueuedCmdCurrentIndex)
      Logger.loginfo('Realworld environment loaded')
    else:  
      Logger.loginfo('Simulation environment loaded')
        
      self._action_topic = action_topic
      self._client = ProxyActionClient({self._action_topic: MoveGroupAction})

      self._move_group = move_group
      self._joint_names = joint_names
            
  def execute(self, userdata):
    '''
    Execute this state
    '''
    if self._planning_failed:
        return 'planning_failed'
    if self._control_failed:
        return 'control_failed'
    if self._success:
        return 'reached'

    if(self._real_world):
      request = GetQueuedCmdCurrentIndexRequest()
      response = self.GetQueuedCmdCurrentIndex_srv(request)
      if(response.result):
        self._control_failed = True
      else:
        if response.queuedCmdCurrentIndex >= self._queuedCmdIndex:
          self._success = True
      return    
    else:
      if self._client.has_result(self._action_topic):
        result = self._client.get_result(self._action_topic)
        
        if result.error_code.val == MoveItErrorCodes.CONTROL_FAILED:
          Logger.logwarn('Control failed for move action of group: %s (error code: %s)' % (self._move_group, str(result.error_code)))
          self._control_failed = True
          return 'control_failed'
        elif result.error_code.val != MoveItErrorCodes.SUCCESS:
          Logger.logwarn('Move action failed with result error code: %s' % str(result.error_code))
          self._planning_failed = True
          return 'planning_failed'
        else:
          self._success = True
          return 'reached'

            
  def on_enter(self, userdata):
    if(self._real_world):

      ptp_command = SetPTPCmdRequest()
      ptp_command.ptpMode = 4
      ptp_command.x = 0
      ptp_command.y = 0
      ptp_command.z = 0
      ptp_command.r = 0
      for i in range(len(userdata.joint_names)):
        if(userdata.joint_names[i] == "magician_joint1"):
          ptp_command.x = math.degrees(userdata.joint_values[i])
        if(userdata.joint_names[i] == "magician_joint2"):
          ptp_command.y = math.degrees(userdata.joint_values[i])
        if(userdata.joint_names[i] == "magician_joint3"):
          ptp_command.z = math.degrees(userdata.joint_values[i])
        if(userdata.joint_names[i] == "magician_joint4"):
          ptp_command.r = math.degrees(userdata.joint_values[i])
          
      response = self.SetPTPCmd_srv(ptp_command)
      if(response.result):
        self._control_failed = True
      self._queuedCmdIndex = response.queuedCmdIndex

    
    else:
      self._planning_failed = False
      self._control_failed = False
      self._success = False

      action_goal = MoveGroupGoal()
      action_goal.request.group_name = self._move_group
      goal_constraints = Constraints()
      for i in range(len(userdata.joint_names)):
        goal_constraints.joint_constraints.append(JointConstraint(joint_name=userdata.joint_names[i], position=userdata.joint_config[i]))
      action_goal.request.goal_constraints.append(goal_constraints)

      try:
        self._client.send_goal(self._action_topic, action_goal)
      except Exception as e:
        Logger.logwarn('Failed to send action goal for group: %s\n%s' % (self._move_group, str(e)))
        self._planning_failed = True
            

  def on_stop(self):
    if(self._real_world):
      pass
    else:
      try:
        if self._client.is_available(self._action_topic) \
        and not self._client.has_result(self._action_topic):
            self._client.cancel(self._action_topic)
      except:
        # client already closed
        pass

  def on_pause(self):
    if(self._real_world):
      pass
    else:
        self._client.cancel(self._action_topic)

  def on_resume(self, userdata):
    if(self._real_world):
      pass
    else:
          self.on_enter(userdata)
