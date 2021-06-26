#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint



def talker():
  pub = rospy.Publisher('/magician_arm_controller/command', JointTrajectory, queue_size=1)
  rospy.init_node('talker', anonymous=True)
  rate = rospy.Rate(1) # 1hz

  joint_trajectory = JointTrajectory()
  joint_trajectory.header.seq = 0
  joint_trajectory.header.stamp = rospy.Time.now()
  joint_trajectory.header.frame_id = 'position_test'
  joint_trajectory.joint_names = ['magician_joint1',  'magician_joint2', 'magician_joint3']
  joint_trajectory.points.append(JointTrajectoryPoint())
  joint_trajectory.points[0].positions = [0.5, 1.0, 1.5]
  joint_trajectory.points[0].velocities = [0, 0, 0]    
  joint_trajectory.points[0].accelerations = [0, 0, 0]    
  joint_trajectory.points[0].effort = [0, 0, 0]
  joint_trajectory.points[0].time_from_start = rospy.Duration(5.0)

  rospy.loginfo(joint_trajectory)
  pub.publish(joint_trajectory)
  rate.sleep()

  #while not rospy.is_shutdown():

  # rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
