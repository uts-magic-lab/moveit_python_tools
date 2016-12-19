#!/usr/bin/env python

import rospy
from pr2_get_ik import GetIK
from go_to_pose import GoToPose
from tf_for_me import TF_stuff
import sys
from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped
from tf.transformations import quaternion_from_euler
from math import degrees, radians
from copy import deepcopy

from moveit_msgs.msg import MoveItErrorCodes
# Build a useful mapping from MoveIt error codes to error names
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

"""
Using an aruco marker, having launched
roslaunch aruco_ros single_pr2_0.83.launch
To detect the printed 0.083m marker

It asks for an IK for the pose
Then executes the trajectory.

First approaches button, then pushes, then goes back.

Author: Sammy Pfeiffer
"""


class ArucoPresser(object):
    def __init__(self):
        rospy.loginfo("ArucoPresser starting...")
        self.last_aruco = None
        self.aruco_sub = rospy.Subscriber('/aruco_single/pose',
                                          PoseStamped, self.aruco_cb,
                                          queue_size=1)
        self.tf_for_me = TF_stuff()
        self.gik = GetIK()
        self.gtp = GoToPose()
        rospy.loginfo("Initialized!")

    def aruco_cb(self, data):
        self.last_aruco = data

    def try_ik_and_go(self, ps, time):
        rospy.loginfo("Trying without torso...")
        if self.compute_ik_and_go(ps, time / 2.0, use_torso=False):
            return True
        else:
            rospy.loginfo("Failed, trying with torso.")
            return self.compute_ik_and_go(ps, time, use_torso=True)

    def compute_ik_and_go(self, ps, time, use_torso=False):
        rospy.loginfo("Getting IK!")
        if use_torso:
            resp = self.gik.get_ik(ps, group='right_arm_and_torso', duration=3.0,
                                   attempts=0)
        else:
            resp = self.gik.get_ik(ps, group='right_arm', duration=3.0,
                                   attempts=0)
        # rospy.loginfo(str(resp))
        err_code = resp.error_code.val
        rospy.loginfo("Error is: " + moveit_error_dict[err_code])

        if moveit_error_dict[err_code] == "SUCCESS":
            rospy.loginfo("Going to pose!")
            joint_names = list(resp.solution.joint_state.name)
            positions = list(resp.solution.joint_state.position)

            def get_ids_of_non_controllable_joints(joint_names):
                ids = []
                bad_joints = ['bl_caster_rotation_joint', 'bl_caster_l_wheel_joint',
                              'bl_caster_r_wheel_joint', 'br_caster_rotation_joint',
                              'br_caster_l_wheel_joint', 'br_caster_r_wheel_joint',
                              'fl_caster_rotation_joint', 'fl_caster_l_wheel_joint',
                              'fl_caster_r_wheel_joint', 'fr_caster_rotation_joint',
                              'fr_caster_l_wheel_joint', 'fr_caster_r_wheel_joint',
                              'torso_lift_motor_screw_joint', 'laser_tilt_mount_joint'
                              ]
                for j in bad_joints:
                    idx = joint_names.index(j)
                    if idx != -1:
                        ids.append(idx)
                return ids

            indexes = get_ids_of_non_controllable_joints(joint_names)
            for index in sorted(indexes, reverse=True):
                del joint_names[index]
                del positions[index]

            self.gtp.go_to_pose(joint_names, positions, time)
            # rospy.loginfo(str(resp))
            err_code = resp.error_code.val
            rospy.loginfo("Error is: " + moveit_error_dict[err_code])
            return True
        else:
            return False

    def press_aruco(self):
        while not rospy.is_shutdown():
            if self.last_aruco is None:
                rospy.loginfo("No aruco...")
                rospy.sleep(1.0)
            else:
                # Go to pre-pose
                p_base_link = self.tf_for_me.transform_pose(self.last_aruco,
                                                            self.last_aruco.header.frame_id,
                                                            'base_link')
                rospy.loginfo("aruco in base_link: " +
                              str(p_base_link.pose.position))
                ps = PoseStamped()
                ps.header.frame_id = 'base_link'
                ps.pose.position = deepcopy(p_base_link.pose.position)
                ps.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

                gripper_long = 0.18
                ps.pose.position.x -= gripper_long
                ps.pose.position.x -= 0.02  # button is out
                ps.pose.position.z += 0.11  # button is under marker

                rospy.loginfo("Adjusting for gripper length and under button:\n" +
                              str(ps.pose.position))
                ps_pre_pose = deepcopy(ps)
                ps_pre_pose.pose.position.x -= 0.06

                if self.try_ik_and_go(ps_pre_pose, 5.0):
                    ps_push_button = deepcopy(ps)
                    ps_push_button.pose.position.x += 0.01
                    rospy.loginfo("Pressing at: " +
                                  str(ps_push_button.pose.position))
                    # Press
                    if self.try_ik_and_go(ps_push_button, 3.0):
                        # Go back
                        rospy.loginfo("Going back at: " +
                                      str(ps_pre_pose.pose.position))
                        if self.try_ik_and_go(ps_pre_pose, 3.0):
                            rospy.loginfo("We are done!!")
                        else:
                            rospy.logwarn("Failed on going back...")
                    else:
                        rospy.logwarn("Failed on press button...")
                else:
                    rospy.logwarn("Failed on pre pose...")
                break


if __name__ == '__main__':
    print "Connecting to ROS node..."
    rospy.init_node('get_ik_and_go')
    ap = ArucoPresser()
    ap.press_aruco()
