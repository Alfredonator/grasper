#! /usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import moveit_grasps
from geometry_msgs.msg import Pose
from moveit_grasps.srv import Grasp
from std_msgs.msg import Bool
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene

class BigBlackBalls:
    robot = None
    arm_group = None
    pub_grip = None

    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.arm_group = moveit_commander.MoveGroupCommander("edo")
        self.ee_group = moveit_commander.MoveGroupCommander("edo_gripper")
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

    def pick_object_by_name(self, object_name):
        rospy.wait_for_service('grasp_pipeline')
        try:
            grasp_pipeline_fn = rospy.ServiceProxy('grasp_pipeline', Grasp)
            robot_poses = grasp_pipeline_fn(object_name)
            self.open_gripper()
            self.pose_go(robot_poses.pre_grasp)
            self.pose_go(robot_poses.grasp)
            rospy.sleep(1)
            self.close_gripper()
            self.attach_object_ee(object_name)
            self.pose_go(robot_poses.pre_grasp)
            self.home_position()
            self.detach_object_ee(object_name)
            # TODO we put this in his mf bucket

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def pose_go(self, pose):
        self.arm_group.set_pose_target(pose)
        self.arm_group.go()

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

    def attach_object_ee(self, object_name):
        links = [
            "edo_gripper_left_base_link",
            "edo_gripper_right_base_link",
            "edo_gripper_right_finger_link",
            "edo_gripper_left_finger_link",
        ]

        self.ee_group.attach_object(object_name, touch_links=links)

    def detach_object_ee(self, object_name):
        self.ee_group.detach_object(object_name)

def main(args):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ball_grasper', anonymous=True)
    bbb = BigBlackBalls()
    bbb.pick_object_by_name("mf")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    rospy.loginfo("Starting ball grasper node")
    main(sys.argv)
rospy.sleep(5)
moveit_commander.roscpp_shutdown()
