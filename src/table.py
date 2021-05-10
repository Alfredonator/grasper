#! /usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg


def add_robot_environment():
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    rospy.sleep(2)
    # create wall
    p = geometry_msgs.msg.PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.17
    p.pose.position.y = -0.05
    p.pose.position.z = -0.02
    # check this site for quaternion orientations https://quaternions.online/
    p.pose.orientation.x = 0.707
    p.pose.orientation.y = -0.707
    p.pose.orientation.z = 0.0
    p.pose.orientation.w = 0.0
    scene.add_box("table", p, (0.85, 1.20, 0.03))


def main(args):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('table_init', anonymous=True)
    add_robot_environment()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    rospy.loginfo("Starting table init node")
    main(sys.argv)
rospy.sleep(5)
moveit_commander.roscpp_shutdown()
