#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension


def talker():
    pub = rospy.Publisher('MagicianHWInterface/magician_arm_controller/command', Float64MultiArray, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz

    robot_joint_position = Float64MultiArray()
    robot_joint_position.data = []


    #while not rospy.is_shutdown():
    robot_joint_position.data = [0.5, 1.0, 1.5]
    robot_joint_position.layout.dim.append(MultiArrayDimension())
    robot_joint_position.layout.dim[0].size=3
    robot_joint_position.layout.dim[0].stride = 1
    robot_joint_position.layout.dim[0].label='positions'
    rospy.loginfo(robot_joint_position)
    pub.publish(robot_joint_position)
    rate.sleep()

    #while not rospy.is_shutdown():

    # rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
