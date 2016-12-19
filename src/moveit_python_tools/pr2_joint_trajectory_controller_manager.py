#!/usr/bin/env python

# Standard ROS imports
import rospy
from actionlib import SimpleActionClient
# Messages
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# This specific controller manager needed messages
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from pr2_controllers_msgs.msg import JointTrajectoryControllerState


"""
Class to manage the connection to a controller
of type pr2_controllers to keep track of the state
and send trajectories to the joint_trajectory_action
interface.

Author: Sammy Pfeiffer
"""


class PR2JointTrajectoryControllerManager(object):
    def __init__(self, namespace):
        """
        Setup connection to ``namespace/joint_trajectory_action`` action server
        and ``namespace/state`` state topic.

        :param str namespace: namespace for the controller from where
            the action server and the state topic hang, e.g. /head_controller
            /head_controller/follow_joint_trajectory/[goal/feedback/cancel/result]
            /head_controller/state
        """
        self.ns = namespace
        self.goal_done = False
        self.last_state = None
        self.state_topic = self.ns + '/state'
        self._state_sub = rospy.Subscriber(self.state_topic,
                                           JointTrajectoryControllerState,
                                           self.state_cb,
                                           # Only the last state is important
                                           queue_size=1)

        self.as_name = self.ns + '/follow_joint_trajectory'
        self._ac = SimpleActionClient(self.as_name,
                                      FollowJointTrajectoryAction)
        self.connect_as()

        self.wait_for_state()
        self.managed_joints = self.last_state.joint_names

    def connect_as(self):
        """
        Connect to the action server printing warnings while
        it's waiting.
        """
        rospy.loginfo("JointTrajectoryControllerManager: connecting to AS '" +
                      str(self.as_name) + "'")
        while not rospy.is_shutdown() and not self._ac.wait_for_server(rospy.Duration(5.0)):
            rospy.logwarn("Waiting for AS '" + self.as_name + "'...")

    def wait_for_state(self):
        """
        Wait for the first state message.
        """
        rospy.loginfo("Waiting for first state on: '" +
                      self.state_topic + "'...")
        while not rospy.is_shutdown() and self.last_state is None:
            rospy.sleep(2.0)
            rospy.logwarn("Waiting for first state on: '" +
                          self.ns + "/state'...")

    def state_cb(self, state):
        """
        Save last state received.

        :param JointTrajectoryControllerState state: last state.
        """
        self.last_state = state

    def send_goal(self, goal):
        """
        Send goal to the controller.

        :param JointTrajectory goal: Goal to send.
        """
        self.goal_done = False
        self.last_result = None
        self.last_feedback = None
        # Check which of our joints the goal has
        joints_in_goal = []
        for j in goal.joint_names:
            if j in self.managed_joints:
                joints_in_goal.append(j)

        rospy.logdebug("This controller manages the joints: " +
                      str(self.managed_joints))
        rospy.logdebug("Goal contains our joints: " + str(joints_in_goal))
        missing_joints = list(set(self.managed_joints) - set(joints_in_goal))
        rospy.logdebug("Goal is missing joints: " + str(missing_joints))
        extra_joints = list(set(goal.joint_names) - set(joints_in_goal))
        rospy.logdebug("Discarding joints not from this controller: " +
                      str(extra_joints))
        if len(extra_joints) > 0:
            # Create a new goal only with the found joints, in the next
            # step it will be filled with any missing joints
            jt = JointTrajectory()
            jt.joint_names = joints_in_goal

            for jp in goal.points:
                p = JointTrajectoryPoint()
                p.time_from_start = jp.time_from_start
                for jname in joints_in_goal:
                    idx = goal.joint_names.index(jname)
                    p.positions.append(jp.positions[idx])
                    if jp.velocities:
                        p.velocities.append(jp.velocities[idx])
                    if jp.accelerations:
                        p.accelerations.append(jp.accelerations[idx])
                    if jp.effort:
                        p.effort.append(jp.effort[idx])
                jt.points.append(p)

            goal = jt

        if len(missing_joints) == len(self.managed_joints):
            # This goal contains no joints of this controller
            # we don't do anything
            rospy.loginfo("Goal contains no joints of this controller.")
            return

        if len(missing_joints) > 0:
            # Fill message for the real controller with the missing joints
            # Transform tuple fields into lists
            goal.joint_names = list(goal.joint_names)
            for point in goal.points:  # type: JointTrajectoryPoint
                point.positions = list(point.positions)
                if point.velocities:
                    point.velocities = list(point.velocities)
                if point.accelerations:
                    point.accelerations = list(point.accelerations)
                if point.effort:
                    point.effort = list(point.effort)

            # Now add the missing joints
            for jname in missing_joints:
                goal.joint_names.append(jname)
                jidx = self.managed_joints.index(jname)
                # TODO: remove this if-assert (it's redundant)
                if jidx == -1:
                    rospy.logerror("Couldn't find joint " + jname +
                                   ", which we should be controlling," +
                                   " in our managed joints.")
                    assert False
                for point in goal.points:
                    point.positions.append(
                        self.last_state.actual.positions[jidx])
                    if point.velocities:
                        point.velocities.append(
                            self.last_state.actual.velocities[jidx])
                    if self.last_state.actual.accelerations:
                        point.accelerations.append(
                            self.last_state.actual.accelerations[jidx])
                    # TODO: deal with forces, they aren't published in state...
                    # but they are in joint_states

        # Now the trajectory should be complete< and we can send the goal
        jtg = FollowJointTrajectoryGoal()
        jtg.trajectory = goal
        rospy.logdebug("Sending goal: " + str(goal))
        self._ac.send_goal(jtg,
                           done_cb=self.done_cb,
                           feedback_cb=self.feedback_cb)

    def get_managed_joints(self):
        """
        Convenience method, return list of managed joints.

        :returns list: list of strings of managed joints.
        """
        return self.managed_joints

    def is_goal_done(self):
        """
        Convenience method, return true if the last goal is done.

        :returns bool: True if goal done, False otherwise.
        """
        return self.goal_done

    def get_result(self):
        """
        Convenience method, returns result.

        :returns FollowJointTrajectoryResult: result.
        """
        return self.last_result

    def done_cb(self, state, result):
        """
        :param int state: int as state actionlib_msgs/GoalStatus.
        :param FollowJointTrajectoryResult result: result of the action server.
        """
        self.goal_done = True
        self.last_result = result

    def get_feedback(self):
        """
        Convenience method, get feedback.

        :returns FollowJointTrajectoryFeedback: feedback.
        """
        return self.last_feedback

    def feedback_cb(self, feedback):
        """
        :param FollowJointTrajectoryFeedback feedback: feedback message.
        """
        self.last_feedback = feedback

    def cancel_all_goals(self):
        """
        Cancel all current goals if any.
        """
        self._ac.cancel_all_goals()

# TODO: Add checking if action server is still alive
# and manage reconnection (I think this is not automatic)


if __name__ == '__main__':
    rospy.init_node('test_jtcm')
    # Launch PR2 simulation
    # roslaunch pr2_gazebo pr2_empty_world.launch
    jtcm = PR2JointTrajectoryControllerManager('/head_traj_controller')
    rospy.sleep(2)

    # Try to send a goal with all joints
    jt = JointTrajectory()
    jt.joint_names = ['head_pan_joint', 'head_tilt_joint']
    p = JointTrajectoryPoint()
    for j in jt.joint_names:
        p.positions.append(0.3)
        p.velocities.append(0.1)
    p.time_from_start = rospy.Duration(1.0)
    jt.points.append(p)

    rospy.loginfo("Sending goal to both joints")
    jtcm.send_goal(jt)
    rospy.sleep(1.0)

    # Try to send a goal with extra joints that dont pertain
    from copy import deepcopy
    jt2 = deepcopy(jt)
    jt2.joint_names.append('non_existing_joint')
    jt2.points[0].positions[0] = -0.3
    jt2.points[0].positions[1] = -0.3
    jt2.points[0].positions.append(0.1)

    rospy.loginfo("Sending goal with one non controlled joint")
    jtcm.send_goal(jt2)
    rospy.loginfo("Sleeping checking is_goal_gone")
    while not jtcm.is_goal_done():
        rospy.loginfo("Feedback: " + str(jtcm.get_feedback()))
        rospy.sleep(0.1)
    rospy.loginfo("Result: " + str(jtcm.get_result()))

    # Try to send a goal missing joints
    jt3 = JointTrajectory()
    jt3.joint_names = ['head_pan_joint']
    p = JointTrajectoryPoint()
    for j in jt3.joint_names:
        p.positions.append(0.3)
        # p.velocities.append(0.0)
    p.time_from_start = rospy.Duration(1.0)
    jt3.points.append(p)

    rospy.loginfo("Sending goal having one missing joint")
    jtcm.send_goal(jt3)
    rospy.sleep(1.0)
    while not jtcm.is_goal_done():
        rospy.sleep(0.1)
    rospy.loginfo("Result: " + str(jtcm.get_result()))

    # Try to send a goal with no joints that pertain
    jt4 = JointTrajectory()
    jt4.joint_names = ['r_shoulder_pan_joint', 'r_upper_arm_roll_joint',
                       'r_shoulder_lift_joint', 'r_forearm_roll_joint',
                       'r_elbow_flex_joint', 'r_wrist_flex_joint',
                       'r_wrist_roll_joint',
                       'l_shoulder_pan_joint', 'l_upper_arm_roll_joint',
                       'l_shoulder_lift_joint', 'l_forearm_roll_joint',
                       'l_elbow_flex_joint', 'l_wrist_flex_joint',
                       'l_wrist_roll_joint',
                       'torso_lift_joint']
    p = JointTrajectoryPoint()
    for j in jt4.joint_names:
        p.positions.append(0.3)
    p.time_from_start = rospy.Duration(1.0)
    jt4.points.append(p)

    rospy.loginfo("Sending goal with no joint that pertains")
    jtcm.send_goal(jt4)
    rospy.sleep(1.0)

    # Send long goal and cancel it midway
    jt5 = JointTrajectory()
    jt5.joint_names = ['head_pan_joint']
    p = JointTrajectoryPoint()
    p.positions.append(-1.0)
    p.time_from_start = rospy.Duration(1.0)
    jt5.points.append(p)
    p2 = JointTrajectoryPoint()
    p2.positions.append(1.0)
    p2.time_from_start = rospy.Duration(1.0)
    jt5.points.append(p2)

    rospy.loginfo("Sending long goal and cancelling midway")
    jtcm.send_goal(jt5)
    rospy.sleep(1.0)
    jtcm.cancel_all_goals()
    rospy.loginfo("Cancelled.")
    rospy.loginfo("Result: " + str(jtcm.get_result()))

    # jt.joint_names = ['head_pan_joint', 'head_tilt_joint',
    #                   'r_shoulder_pan_joint', 'r_upper_arm_roll_joint',
    #                   'r_shoulder_lift_joint', 'r_forearm_roll_joint',
    #                   'r_elbow_flex_joint', 'r_wrist_flex_joint',
    #                   'r_wrist_roll_joint',
    #                   'l_shoulder_pan_joint', 'l_upper_arm_roll_joint',
    #                   'l_shoulder_lift_joint', 'l_forearm_roll_joint',
    #                   'l_elbow_flex_joint', 'l_wrist_flex_joint',
    #                   'l_wrist_roll_joint',
    #                   'torso_lift_joint']
