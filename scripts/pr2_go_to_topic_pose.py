#!/usr/bin/env python

import rospy
from moveit_python_tools.get_ik import GetIK
from moveit_python_tools.go_to_configuration import GoToConfiguration
from geometry_msgs.msg import PoseStamped
from moveit_python_tools.friendly_error_codes import moveit_error_dict

"""
Give it a pose stamped thru a topic
and the robot will get and IK and execute a
trajectory to it.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


class GoToTopicPose(object):
    def __init__(self, group, time_for_goals=0.1):
        # TODO: Decide duration of motion relative to how far goals are
        rospy.loginfo("Initializing GoToTopicPose...")
        self.group_name = group
        self.time_for_goals = time_for_goals
        self.gik = GetIK(self.group_name)
        self.gtp = GoToConfiguration()
        self.sub = rospy.Subscriber('~goal_pose',
                                    PoseStamped,
                                    self.pose_cb,
                                    queue_size=1)
        rospy.loginfo("Listening for PoseStamped goals at: " +
                      str(self.sub.resolved_name))

    def pose_cb(self, ps):
        rospy.loginfo("Getting IK for pose: " + str(ps))
        resp = self.gik.get_ik(ps,
                               group=self.group_name,
                               ik_timeout=0.5,
                               ik_attempts=0)
        rospy.loginfo("IK was: " + str(resp))
        err_code = resp.error_code.val
        rospy.loginfo("Error is: " + moveit_error_dict[err_code])

        if moveit_error_dict[err_code] == "SUCCESS":
            rospy.loginfo("Going to pose!")
            joint_names = list(resp.solution.joint_state.name)
            positions = list(resp.solution.joint_state.position)

            def get_ids_of_non_controllable_joints(joint_names):
                ids = []
                # Non controllable joints = no_j
                no_j = ['bl_caster_rotation_joint', 'bl_caster_l_wheel_joint',
                        'bl_caster_r_wheel_joint', 'br_caster_rotation_joint',
                        'br_caster_l_wheel_joint', 'br_caster_r_wheel_joint',
                        'fl_caster_rotation_joint', 'fl_caster_l_wheel_joint',
                        'fl_caster_r_wheel_joint', 'fr_caster_rotation_joint',
                        'fr_caster_l_wheel_joint', 'fr_caster_r_wheel_joint',
                        'torso_lift_motor_screw_joint', 'laser_tilt_mount_joint'
                        ]
                for j in no_j:
                    idx = joint_names.index(j)
                    if idx != -1:
                        ids.append(idx)
                return ids

            indexes = get_ids_of_non_controllable_joints(joint_names)
            for index in sorted(indexes, reverse=True):
                del joint_names[index]
                del positions[index]

            resp = self.gtp.go_to_configuration(joint_names, positions,
                                                self.time_for_goals)
            rospy.loginfo(str(resp))
            err_code = resp.error_code.val
            rospy.loginfo("Error is: " + moveit_error_dict[err_code])


if __name__ == '__main__':
    rospy.init_node('pr2_go_to_topic_pose')
    gttp = GoToTopicPose(group="right_arm")
    rospy.spin()
