#!/usr/bin/env python

import rospy
import math
import xml.etree.ElementTree as ET
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, MoveItErrorCodes
from magician_hardware.srv import SetPTPCmd, SetPTPCmdRequest, SetPTPCmdResponse
from magician_hardware.srv import GetQueuedCmdCurrentIndex, GetQueuedCmdCurrentIndexRequest, GetQueuedCmdCurrentIndexResponse

'''
Created on 10.10.2016

@author: Alberto Romay
@modified: Gerard Harkema
'''

class SrdfStateToMagician(EventState):
  '''
  State to look up a pre-defined joint configuration from the SRDF file loaded in the parameter server (/robot_description_semantic)
  and send it to MoveIt to plan and move.

  ># config_name          string              Name of the joint configuration of interest.

  ># move_group           string              Name of the move group to be used for planning.

  ># action_topic_namespace    string              Name of the namespace of the move group to be used for planning.

  ># action_topic         string              Topic on which MoveIt is listening for action calls.

  ># robot_name           string              Optional name of the robot to be used.
                                                          If left empty, the first one found will be used
                                                          (only required if multiple robots are specified in the same file).

  #> joint_names		string[]	    Names of the joints to set.
					  Does not need to specify all joints.
  #> joint_values		float[]		    Target configuration of the joints.
					  Same order as their corresponding names in joint_names.
  <= reached                                  Target joint configuration has been reached.
  <= planning_failed                          Failed to find a plan to the given joint configuration.
  <= control_failed                           Failed to move the arm along the planned trajectory.

  '''

  def __init__(self):
    '''
    Constructor
    '''
    super(SrdfStateToMagician, self).__init__(outcomes=['reached', 'planning_failed', 'control_failed', 'param_error'],
    input_keys = ['config_name', 'move_group', 'action_topic_namespace', 'action_topic', 'robot_name'],
                  output_keys=['config_name_out', 'move_group_out', 'robot_name_out',  'action_topic_out', 'joint_values', 'joint_names'])


    self._planning_failed = False
    self._control_failed = False
    self._success = False

    self._param_error = False
    self._srdf = None

    self.service_timeout = 0.5

    self._real_world = True
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
      
  def execute(self, userdata):
    if self._param_error:
      return 'param_error'
    '''
    Execute this state
    '''
    if self._planning_failed:
      return 'planning_failed'
    if self._control_failed:
      return 'control_failed'
    if self._success:
      return 'reached'
    
    if self._real_world:
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
      self._param_error     = False
      self._planning_failed = False
      self._control_failed  = False
      self._success         = False

      self._config_name  = userdata.config_name
      self._move_group   = userdata.move_group
      self._robot_name   = userdata.robot_name


      self._planning_failed = False
      self._control_failed = False


      self._srdf_param = None
      if rospy.has_param(userdata.action_topic_namespace + '/robot_description_semantic'):
        self._srdf_param = rospy.get_param(userdata.action_topic_namespace + '/robot_description_semantic')
      else:
        Logger.logerr('Unable to get parameter: %s' % srdf_param)
        return

      self._param_error = False
      self._srdf = None


      try:
        self._srdf = ET.fromstring(self._srdf_param)
      except Exception as e:
        Logger.logwarn('Unable to parse given SRDF parameter: /robot_description_semantic')
        self._param_error = True

      if not self._param_error:
        robot = None
        for r in self._srdf.iter('robot'):
          if self._robot_name == '' or self._robot_name == r.attrib['name']:
            robot = r
            userdata.robot_name_out = robot  # Save robot name to output key
            break
        if robot is None:
          Logger.logwarn('Did not find robot name in SRDF: %s' % self._robot_name)
          return 'param_error'

        # get joint names from group
        for g in robot.iter('group'):
          if self._move_group == g.attrib['name']:
            self._joint_names  = [str(j.attrib['name']) for j in g.iter('joint')]
            break

        config = None
        for c in robot.iter('group_state'):
          if (self._move_group == '' or self._move_group == c.attrib['group']) \
            and c.attrib['name'] == self._config_name:
            config = c
            self._move_group = c.attrib['group']  # Set move group name in case it was not defined
            userdata.config_name_out = config           # Save configuration name to output key
            userdata.move_group_out  = self._move_group  # Save move_group to output key
            break
        if config is None:
          Logger.logwarn('Did not find config name in SRDF: %s' % self._config_name)
          return 'param_error'

        # get joint values from specific jount in group_state
        self._joint_config=[]
        for joint_name in self._joint_names:
          for j in config.iter('joint'):
            if j.attrib['name'] == joint_name:
              self._joint_config.append(float(j.attrib['value']))


        userdata.joint_values = self._joint_config  # Save joint configuration to output key
        userdata.joint_names  = self._joint_names  # Save joint names to output key

        '''
        for i in range(len(self._joint_names)):
            Logger.logwarn('Jointname: %s' % self._joint_names[i])
            Logger.logwarn('Jointconfig: %f' % self._joint_config[i])
        '''      
        
        if self._real_world:
          ptp_command = SetPTPCmdRequest()
          ptp_command.ptpMode = 4
          ptp_command.x = 0
          ptp_command.y = 0
          ptp_command.z = 0
          ptp_command.r = 0
          for i in range(len(self._joint_names)):
            if(self._joint_names[i] == "magician_joint1"):
              ptp_command.x = math.degrees(self._joint_config[i])
            if(self._joint_names[i] == "magician_joint2"):
              ptp_command.y = math.degrees(self._joint_config[i])
            if(self._joint_names[i] == "magician_joint3"):
              ptp_command.z = math.degrees(self._joint_config[i])
            if(self._joint_names[i] == "magician_joint4"):
              ptp_command.r = math.degrees(self._joint_config[i])
              
          response = self.SetPTPCmd_srv(ptp_command)
          if(response.result):
            self._control_failed = True
          self._queuedCmdIndex = response.queuedCmdIndex
            
        else:
          self._action_topic = userdata.action_topic_namespace + userdata.action_topic
          self._client       = ProxyActionClient({self._action_topic: MoveGroupAction})
          # Action Initialization
          action_goal = MoveGroupGoal()
          action_goal.request.group_name = self._move_group
          action_goal.request.allowed_planning_time = 1.0
          goal_constraints = Constraints()
          for i in range(len(self._joint_names)):
            goal_constraints.joint_constraints.append(JointConstraint(
              joint_name=self._joint_names[i],
              position=self._joint_config[i],
              weight=1.0))
            #Logger.logwarn('Jointname: %s' % self._joint_names[i])
            #Logger.logwarn('Jointconfig: %f' % self._joint_config[i])
            action_goal.request.goal_constraints.append(goal_constraints)
          try:
            self._client.send_goal(self._action_topic, action_goal)
            userdata.action_topic_out = self._action_topic  # Save action topic to output key
          except Exception as e:
            Logger.logwarn('Failed to send action goal for group: %s\n%s' % (self._move_group, str(e)))
            self._planning_failed = True

  def on_stop(self):
    if self._real_world:
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
    if self._real_world:
      pass
    else:
      self._client.cancel(self._action_topic)

  def on_resume(self, userdata):
    if self._real_world:
      pass
    else:
      self.on_enter(userdata)