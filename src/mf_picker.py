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
        self.arm_group.allow_replanning(True)
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
                    if self._is_box_for_given_object_present(object_to_pickup):
                        print("picking up ", object_to_pickup)
                        self.pick_object_by_name(object_to_pickup)
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
        robot_poses = self._get_grasp_poses_for_object(object_name)

        self.open_gripper()

        rospy.loginfo("Going to pre-grasp position")
        self.go_pose(robot_poses.pre_grasp)

        rospy.loginfo("Going to grasp position")
        self.go_pose(robot_poses.grasp)
        rospy.sleep(1)
        self.attach_object_ee(object_name)
        rospy.loginfo("Closing the grip")
        self.close_gripper()

        rospy.loginfo("Going to pre-grasp position")
        self.go_pose(robot_poses.pre_grasp)

    def place_object_in_bucket(self, object_name):
        box_pose = self._get_box_pose(self._create_box_name_from_object_name(object_name))

        self.go_pose(box_pose)
        self.open_gripper()
        self.detach_object_ee(object_name)

        self.go_home_position()
        return

    def _get_box_pose(self, box_name):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.pose = self._scene.get_object_poses([box_name])[box_name]
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.position.z += 0.16  # to place 30cm above

        small_box_name = box_name + "_grasp"
        small_box_pose = box_pose
        print("small box pose")
        print(small_box_pose)

        self._scene.add_box(small_box_name, small_box_pose, size=(0.03, 0.03, 0.03))
        rospy.sleep(3)
        grasp_pose = self._get_grasp_poses_for_object(small_box_name)
        if grasp_pose is None:
            raise Exception('grasp_pose is null')

        self._scene.remove_world_object(small_box_name)

        return grasp_pose.grasp

    @staticmethod
    def _get_grasp_poses_for_object(object_name):
        rospy.wait_for_service('grasp_pipeline')
        try:
            grasp_pipeline_fn = rospy.ServiceProxy('grasp_pipeline', Grasp)
            robot_poses = grasp_pipeline_fn(object_name)
            print("after grasp pipeline")
            print(robot_poses)
            if robot_poses is None:
                raise Exception('robot poses is null')
            return robot_poses
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    @staticmethod
    def _create_box_name_from_object_name(object_name):
        return "Box_" + object_name

    def go_pose(self, pose):
        self.arm_group.set_joint_value_target(pose, "edo_link_ee", True)
        #self.arm_group.set_pose_target(pose)
        self.arm_group.go(pose)

    def close_gripper(self):
        self.pub_grip.publish(False)

    def open_gripper(self):
        self.pub_grip.publish(True)

    def go_home_position(self):
        self.arm_group.set_named_target("candle")
        self.arm_group.go()

    def attach_object_ee(self, object_name):
        touch_links = self.robot.get_link_names(group="edo_gripper")
        self._scene.attach_box("edo_gripper_link", object_name, touch_links=touch_links)

    def detach_object_ee(self, object_name):
        self._scene.remove_attached_object("edo_gripper_link", name=object_name)
        self._scene.remove_world_object(object_name)

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
