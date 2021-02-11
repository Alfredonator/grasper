#! /usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import Bool


class ball_grsaper:
    robot = None
    arm_group = None
    pub_grip = None

    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.arm_group = moveit_commander.MoveGroupCommander("edo")
        self.pub_grip = rospy.Publisher('open_gripper', Bool, queue_size=10)

    def pick_object(self, x, y, z):
        rospy.loginfo("Picking up object from coordinates %s, %s, %s", x, y, z)
        self.open_gripper()
        self.move_arm(x, y, z + 0.20, 0.017422152127, 0.990489039778, 0.133863815194, -0.0266159665721)
        self.move_arm(x, y, z + 0.11, 0.017422152127, 0.990489039778, 0.133863815194, -0.0266159665721)
        self.close_gripper()
        self.move_arm(0.18, 0.32, 0.28, 0.017422152127, 0.990489039778, 0.133863815194, -0.0266159665721)
        self.open_gripper()
        # self.home_position()
        return 0

    def move_arm(self, p_x, p_y, p_z, o_w, o_x, o_y, o_z):
        pose = self.create_pose(p_x, p_y, p_z, o_w, o_x, o_y, o_z)
        self.arm_group.set_pose_target(pose)
        self.arm_group.go()

    def create_pose(self, p_x, p_y, p_z, o_w, o_x, o_y, o_z):
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = o_w
        pose_target.orientation.x = o_x
        pose_target.orientation.y = o_y
        pose_target.orientation.z = o_z
        pose_target.position.x = p_x
        pose_target.position.y = p_y
        pose_target.position.z = p_z
        return pose_target

    def close_gripper(self):
        self.pub_grip.publish(False)

    def open_gripper(self):
        self.pub_grip.publish(True)

    def home_position(self):
        self.arm_group.set_named_target("candle")
        self.arm_group.go()


class grasper_brain:
    consumer_ball_pose = None
    counter = 0
    previous_pose = [0, 0, 0]
    ball_grasper = None
    robot_working = False

    def __init__(self):
        self.ball_grasper = ball_grsaper()
        self.consumer_ball_pose = rospy.Subscriber("/ball_coordinates", geometry_msgs.msg.Pose, self.decision_making,
                                                   queue_size=10)

    def decision_making(self, pose):
        if self.is_ball_stable(pose) and not self.robot_working:
            self.robot_working = True
            rospy.loginfo("Ordering pickup of object at position %s %s %s", pose.position.x, pose.position.y,
                          pose.position.z)
            self.ball_grasper.pick_object(pose.position.x, pose.position.y, pose.position.z)
            self.robot_working = False

    def is_ball_stable(self, pose):
        # rospy.loginfo("Checking if ball is stable" + pose)
        rospy.loginfo("Checking if ball is stable %s %s %s", pose.position.x, pose.position.y,
                      pose.position.z)
        if self.counter > 120:
            self.counter = 0
            return True
        if (distance_less_3cm(pose.position.x, self.previous_pose[0]) or
                distance_less_3cm(pose.position.y, self.previous_pose[1]) or
                distance_less_3cm(pose.position.y, self.previous_pose[2])):
            self.counter += 1
            self.previous_pose = [pose.position.x, pose.position.y, pose.position.z]
            return False
        else:
            self.counter = 0
            self.previous_pose = [pose.position.x, pose.position.y, pose.position.z]
            return False


def distance_less_3cm(x, y):
    return abs(x - y) < 0.03


def add_robot_environment():
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    rospy.sleep(2)
    # create wall
    p = geometry_msgs.msg.PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = -0.33
    p.pose.position.y = -0.33
    p.pose.position.z = 1
    # check this site for quaternion orientations https://quaternions.online/
    p.pose.orientation.x = 0.966
    p.pose.orientation.y = 0.259
    p.pose.orientation.z = 0.
    p.pose.orientation.w = 0.
    scene.add_box("wall", p, (0.01, 2, 2))

    # create cup
    p.pose.position.x = 0.18
    p.pose.position.y = 0.32
    p.pose.position.z = 0.11
    # check this site for quaternion orientations https://quaternions.online/
    p.pose.orientation.x = 0.
    p.pose.orientation.y = 0.
    p.pose.orientation.z = 0.
    p.pose.orientation.w = 0.
    scene.add_box("bucket", p, (0.08, 0.08, 0.22))


def main(args):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ball_grasper', anonymous=True)
    add_robot_environment()
    grasper_brain()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    rospy.loginfo("Starting ball grasper node")
    main(sys.argv)
rospy.sleep(5)
moveit_commander.roscpp_shutdown()
