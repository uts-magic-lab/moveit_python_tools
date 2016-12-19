#!/usr/bin/env python

import rospy
from moveit_python_tools.get_ik import GetIK
from moveit_python_tools.go_to_configuration import GoToConfiguration
from geometry_msgs.msg import PoseStamped
from moveit_python_tools.friendly_error_codes import moveit_error_dict
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_python_tools.pr2_joint_trajectory_controller_manager import PR2JointTrajectoryControllerManager
import time

"""
Give it a pose stamped thru a topic
and the robot will get and IK and execute a
trajectory to it.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


class GoToTopicPoseByFJT(object):
    def __init__(self, side, time_for_goals=0.25):
        # TODO: Decide duration of motion relative to how far goals are
        rospy.loginfo("Initializing GoToTopicPose...")
        self.time_for_goals = time_for_goals
        if side == 'right':
            self.group_name = 'right_arm'
            self.gik = GetIK(self.group_name)
            self.arm_fjt = PR2JointTrajectoryControllerManager(
                '/r_arm_controller')
            self.pose_topic = '/right_arm_goal_pose'
        elif side == 'left':
            self.group_name = 'left_arm'
            self.gik = GetIK(self.group_name)
            self.arm_fjt = PR2JointTrajectoryControllerManager(
                '/l_arm_controller')
            self.pose_topic = '/left_arm_goal_pose'

        self.sub = rospy.Subscriber(self.pose_topic,
                                    PoseStamped,
                                    self.pose_cb,
                                    queue_size=1)
        rospy.loginfo("Listening for PoseStamped goals at: " +
                      str(self.sub.resolved_name))

    def pose_cb(self, ps):
        rospy.loginfo("Getting IK for pose: " + str(ps))
        ini_t = time.time()
        resp = self.gik.get_ik(ps,
                               group=self.group_name,
                               ik_timeout=0.5,
                               ik_attempts=0)
        fin_t = time.time()
        dur_ik = fin_t - ini_t
        rospy.logwarn("IK call took: " + str(dur_ik))
        # rospy.loginfo("IK was: " + str(resp))
        err_code = resp.error_code.val
        rospy.loginfo("Error is: " + moveit_error_dict[err_code])

        if moveit_error_dict[err_code] == 'SUCCESS':
            joint_trajectory = self.to_joint_trajectory(resp)
            self.arm_fjt.send_goal(joint_trajectory)
        else:
            rospy.logwarn("No IK, error: " + moveit_error_dict[err_code])

    def to_joint_trajectory(self, resp):
        """
        Transform from GetPosititionIKResponse
        to JointTrajectory
        """
        jt = JointTrajectory()
        jtp = JointTrajectoryPoint()
        # resp.solution is a RobotState
        jst = resp.solution.joint_state
        jt.joint_names = jst.name
        for i in range(len(jst.name)):
            jtp.positions.append(jst.position[i])
            if jst.velocity:
                jtp.velocities.append(jst.velocity[i])
            if jst.effort:
                jtp.effort.append(jst.effort[i])
        jtp.time_from_start = rospy.Duration(self.time_for_goals)
        jt.points.append(jtp)
        return jt


if __name__ == '__main__':
    rospy.init_node('pr2_go_to_topic_pose')
    gttpfjt = GoToTopicPoseByFJT(side="left")
    gttpfjt = GoToTopicPoseByFJT(side="right")
    rospy.spin()
