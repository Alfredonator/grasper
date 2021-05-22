#! /usr/bin/env python
import random
import sys
import time

import rospy
import moveit_commander
import geometry_msgs.msg
import moveit_grasps
from geometry_msgs.msg import Pose
from moveit_grasps.srv import Grasp
from std_msgs.msg import Bool


class Brain:
    robot = None
    arm_group = None
    pub_grip = None
    OBJECTS_LIST = ["Potato", "Lemon", "Radish"]

    def __init__(self):
        self._scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.arm_group = moveit_commander.MoveGroupCommander("edo")
        self.ee_group = moveit_commander.MoveGroupCommander("edo_gripper")
        self.pub_grip = rospy.Publisher('open_gripper', Bool, queue_size=10)
        self.operate()

    def operate(self):
        """
        get objects and boxes
        randomly select one
        if box is present for a kind of object:
            pick object
            place in a bucket
            return home
        """
        try:
            while True:
                object_to_pickup = self.select_object_to_pickup()
                if object_to_pickup:
                    if True: #self._is_box_for_given_object_present(object_to_pickup):
                        print("picking up ", object_to_pickup)
                        self.pick_object_by_name(object_to_pickup)
                        object_to_pickup = "Potato"
                        self.place_object_in_bucket(object_to_pickup)
                    else:
                        print("No box for present for " + object_to_pickup + ", aborting")
                else:
                    print("No object found!")
                time.sleep(2)

        except KeyboardInterrupt:
            print("Shutdown")

    def select_object_to_pickup(self):
        planning_scene_object_list = self._scene.get_known_object_names()
        present_detected_objects = []

        for scene_object in planning_scene_object_list:
            if self.OBJECTS_LIST.__contains__(scene_object):
                present_detected_objects.append(scene_object)

        if len(present_detected_objects) == 0:
            return None
        else:
            return present_detected_objects[random.randint(0, len(present_detected_objects)-1)]

    def _is_box_for_given_object_present(self, object_name):
        planning_scene_object_list = self._scene.get_known_object_names()

        return planning_scene_object_list.__contains__(self._create_box_name_from_object_name(object_name))

    def pick_object_by_name(self, object_name):
        rospy.wait_for_service('grasp_pipeline')
        try:
            grasp_pipeline_fn = rospy.ServiceProxy('grasp_pipeline', Grasp)
            robot_poses = grasp_pipeline_fn(object_name)

            self.open_gripper()
            rospy.sleep(1)

            self.go_pose(robot_poses.pre_grasp)
            self.go_pose(robot_poses.grasp)

            self.close_gripper()
            rospy.sleep(1)
            #self.attach_object_ee(object_name)

            self.go_pose(robot_poses.pre_grasp)
            self.go_home_position()

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def place_object_in_bucket(self, object_name):
        box_pose = self._get_box_pose(self._create_box_name_from_object_name(object_name))
        print("box pose before z increase: ", box_pose)
        box_pose.position.z += 0.30  # to place gripper 25cm above
        box_pose.orientation.w = 0.017422152127
        box_pose.orientation.x = 0.990489039778
        box_pose.orientation.y = 0.133863815194
        box_pose.orientation.z = -0.0266159665721
        print("box pose AFTER z increase: ", box_pose)

        self.go_pose(box_pose)

        self.open_gripper()
        rospy.sleep(1)
        self.detach_object_ee(object_name)

        self.go_home_position()
        return

    def _get_box_pose(self, box_name):

        return self._scene.get_object_poses([box_name])[box_name]

    @staticmethod
    def _create_box_name_from_object_name(object_name):
        return "Box_" + object_name

    def go_pose(self, pose):
        #self.arm_group.set_joint_value_target(pose, "edo_link_ee", True)
        # self.arm_group.set_pose_target(pose)
        #self.arm_group.set_planning_time(10)
        self.arm_group.go(pose)

    def close_gripper(self):
        self.pub_grip.publish(False)

    def open_gripper(self):
        self.pub_grip.publish(True)

    def go_home_position(self):
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


if __name__ == '__main__':
    rospy.init_node('ball_grasper', anonymous=True)
    rospy.loginfo("Starting ball grasper node")
    moveit_commander.roscpp_initialize(sys.argv)
    try:
        Brain()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    moveit_commander.roscpp_shutdown()


"""
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
        """