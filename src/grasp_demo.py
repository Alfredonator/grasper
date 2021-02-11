#! /usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import Bool


def try_method (pose):
    rospy.loginfo(pose)


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('edo_grasper', anonymous=True)
robot = moveit_commander.RobotCommander()

arm_group = moveit_commander.MoveGroupCommander("edo")

world = arm_group.get_current_pose()
rospy.loginfo(world)

pub = rospy.Publisher('open_gripper', Bool, queue_size=10)
ball_pose = rospy.Subscriber("/ball_coordinates", geometry_msgs.msg.Pose, try_method)
rospy.spin()

pub.publish(True)
rospy.sleep(3)

arm_group.set_named_target("candle")
arm_group.go()

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 0.017422152127
pose_target.orientation.x = 0.990489039778
pose_target.orientation.y = 0.133863815194
pose_target.orientation.z = -0.0266159665721
pose_target.position.x = 0.361
pose_target.position.y = 0.157
pose_target.position.z = 0.22

arm_group.set_pose_target(pose_target)
arm_group.go()

pose_target.orientation.w = 0.017422152127
pose_target.orientation.x = 0.990489039778
pose_target.orientation.y = 0.133863815194
pose_target.orientation.z = -0.0266159665721
pose_target.position.x = 0.361
pose_target.position.y = 0.157
pose_target.position.z = 0.13

arm_group.set_pose_target(pose_target)
arm_group.go()

pub.publish(False)
rospy.sleep(1)

pose_target.orientation.w = 0.790328003465
pose_target.orientation.x = -0.494751760276
pose_target.orientation.y = 0.306325483632
pose_target.orientation.z = -0.191747335627
pose_target.position.x = 0.184878366264
pose_target.position.y = 0.292282320389
pose_target.position.z = 0.648695445153

arm_group.set_pose_target(pose_target)
arm_group.go()


rospy.sleep(5)
moveit_commander.roscpp_shutdown()

