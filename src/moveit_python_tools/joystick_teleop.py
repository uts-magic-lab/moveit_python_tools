#! /usr/bin/env python

"""
Multiple classes to teleop using a PS3 controller.

By default:

R1 and R2 are deadman switches.

R1 controls left arm.
R2 controls right arm.

Left stick controls XY pose
Right stick controls Z pose
The arrow buttons control the orientation (roll, pitch).
Select and Start do yaw.

Control grippers with
[R1/R2] + Square/Circle buttons.

Control torso with:
[R1/R2] + Triangle/X buttons.

PS3 button is reset to initial pose.

Triangle and X have no use right now.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import Joy, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pr2_controllers_msgs.msg import Pr2GripperCommand
from moveit_python_tools.get_fk import GetFK
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians
from copy import deepcopy
import time


def msg_to_list(message):
    """
    Given a message, e.g.: Vector3(x=0.0, y=0.0, z=0.0)
    returns the list of it's slots, e.g.: [0.0, 0.0, 0.0]
    :param message: some message that we want it's slots into a list
    :return: the slots as a list
    """
    output_list = []
    for slot in message.__slots__:
        output_list.append(message.__getattribute__(slot))
    return output_list


class JoystickGripper(object):
    def __init__(self, config_dict=None):
        if config_dict is None:
            self.gripper_command_topic = rospy.get_param('~gripper_command_topic',
                                                         '/l_gripper_controller/command')
            self.gripper_joint_name = rospy.get_param('~gripper_joint_name',
                                                      'r_gripper_joint')
            self.joystick_topic = rospy.get_param('~joystick_topic',
                                                  '/joy')
            self.deadman_button = rospy.get_param('~deadman_button', 11)
            self.open_button = rospy.get_param('~open_button', 13)
            self.close_button = rospy.get_param('~close_button', 15)
            self.gripper_step = rospy.get_param('~gripper_step', 0.1)
            self.min_gripper_pos = rospy.get_param('~min_gripper_pos', 0.08)
            self.max_gripper_pos = rospy.get_param('~max_gripper_pos', 1.0)
            self.node_rate_hz = rospy.get_param('~node_rate_hz', 4)
        else:
            self.gripper_command_topic = config_dict['gripper_command_topic']
            self.gripper_joint_name = config_dict['gripper_joint_name']
            self.joystick_topic = config_dict['joystick_topic']
            self.deadman_button = config_dict['deadman_button']
            self.open_button = config_dict['open_button']
            self.close_button = config_dict['close_button']
            self.gripper_step = config_dict['gripper_step']
            self.min_gripper_pos = config_dict['min_gripper_pos']
            self.max_gripper_pos = config_dict['max_gripper_pos']
            self.node_rate_hz = config_dict['node_rate_hz']

        rospy.loginfo("Waiting for first /joint_states message...")
        self.last_js = None
        self.js_sub = rospy.Subscriber('/joint_states', JointState,
                                       self.js_cb, queue_size=1)
        while not rospy.is_shutdown() and self.last_js is None:
            rospy.sleep(0.1)

        self.gripper_idx = self.last_js.name.index(self.gripper_joint_name)
        self.gripper_amount = self.get_current_gripper_position()
        self.last_gripper_amount = self.gripper_amount
        self.gripper_pub = rospy.Publisher(self.gripper_command_topic,
                                           Pr2GripperCommand,
                                           queue_size=1)
        rospy.loginfo("Publishing to: " + self.gripper_pub.resolved_name)

        self.last_joy_stamp = time.time()
        self.joy_sub = rospy.Subscriber(self.joystick_topic,
                                        Joy,
                                        self.joy_cb,
                                        queue_size=1)
        rospy.loginfo("Subscribing to:" + str(self.joy_sub.resolved_name))

    def js_cb(self, data):
        self.last_js = data

    def get_current_gripper_position(self):
        return self.last_js.position[self.gripper_idx]

    def joy_cb(self, data):
        """
        :type data: Joy
        """
        new_t = time.time()
        last_t = self.last_joy_stamp
        if (new_t - last_t) < (1.0 / self.node_rate_hz):
            return

        self.gripper_amount = self.get_current_gripper_position()
        if data.buttons[self.deadman_button]:
            self.gripper_amount += data.buttons[
                self.open_button] * self.gripper_step
            self.gripper_amount += data.buttons[
                self.close_button] * -self.gripper_step
            if self.gripper_amount < self.min_gripper_pos:
                self.gripper_amount = self.min_gripper_pos
            elif self.gripper_amount > self.max_gripper_pos:
                self.gripper_amount = self.max_gripper_pos

            if self.gripper_amount != self.last_gripper_amount:
                cmd = Pr2GripperCommand()
                cmd.max_effort = 100.0
                cmd.position = self.gripper_amount
                self.gripper_pub.publish(cmd)
                rospy.loginfo("Gripper cmd: " + self.gripper_command_topic +
                              " receiving: " + str(cmd))
                self.last_gripper_amount = self.gripper_amount


class JoystickBase(object):
    def __init__(self, config_dict=None):
        pass


class JoystickHead(object):
    def __init__(self, config_dict=None):
        if config_dict is None:
            self.head_command_topic = rospy.get_param('~head_command_topic',
                                                      '/head_controller/command')
            self.joystick_topic = rospy.get_param('~joystick_topic',
                                                  '/joy')
            self.deadman_button_1 = rospy.get_param('~deadman_button_1', 11)
            self.deadman_button_2 = rospy.get_param('~deadman_button_2', 9)
            self.head_up_button = rospy.get_param('~head_up_button', 12)
            self.head_down_button = rospy.get_param('~head_down_button', 14)
            self.head_left_button = rospy.get_param('~head_left_button', 12)
            self.head_right_button = rospy.get_param('~head_right_button', 14)
            self.min_pan_head = rospy.get_param('~min_pan_head', 0.01)
            self.max_pan_head = rospy.get_param('~max_pan_head', 0.30)
            self.min_tilt_head = rospy.get_param('~min_tilt_head', 0.01)
            self.max_tilt_head = rospy.get_param('~max_tilt_head', 0.30)
            self.head_step = rospy.get_param('~head_step', 0.03)
            self.head_timestep = rospy.get_param('~head_timestep', 0.5)
            self.node_rate_hz = rospy.get_param('~node_rate_hz', 4)
        else:
            self.head_command_topic = config_dict['head_command_topic']
            self.joystick_topic = config_dict['joystick_topic']
            self.deadman_button_1 = config_dict['deadman_button_1']
            self.deadman_button_2 = config_dict['deadman_button_2']
            self.head_up_button = config_dict['head_up_button']
            self.head_down_button = config_dict['head_down_button']
            self.head_step = config_dict['head_step']
            self.head_timestep = config_dict['head_timestep']
            self.min_pan_head = config_dict['min_pan_head']
            self.max_pan_head = config_dict['max_pan_head']
            self.min_tilt_head = config_dict['min_tilt_head']
            self.max_tilt_head = config_dict['max_tilt_head']
            self.node_rate_hz = config_dict['node_rate_hz']

        # Start with current head position
        rospy.loginfo("Waiting for first /joint_states message...")
        self.last_js = None
        self.js_sub = rospy.Subscriber('/joint_states', JointState,
                                       self.js_cb, queue_size=1)
        while not rospy.is_shutdown() and self.last_js is None:
            rospy.sleep(0.1)

        self.head_pan_idx = self.last_js.name.index('head_pan_joint')
        self.head_tilt_idx = self.last_js.name.index('head_tilt_joint')
        self.head_position = self.get_current_head_position()
        self.last_head_position = self.head_position
        self.head_pub = rospy.Publisher(self.head_command_topic,
                                        JointTrajectory,
                                        queue_size=1)
        rospy.loginfo("Publishing to: " + str(self.head_pub.resolved_name))

        self.last_joy_stamp = time.time()
        self.joy_sub = rospy.Subscriber(self.joystick_topic,
                                        Joy,
                                        self.joy_cb,
                                        queue_size=1)
        rospy.loginfo("Subscribing to:" + str(self.joy_sub.resolved_name))

    def js_cb(self, data):
        self.last_js = data

    def get_current_head_position(self):
        return self.last_js.position[self.head_pan_idx], self.last_js.position[self.head_tilt_idx]

    def joy_cb(self, data):
        """
        :type data: Joy
        """
        new_t = time.time()
        last_t = self.last_joy_stamp
        if (new_t - last_t) < (1.0 / self.node_rate_hz):
            return

        self.head_position = self.get_current_head_position()
        pan = self.head_position[0]
        if data.buttons[self.deadman_button_1] or data.buttons[self.deadman_button_2]:
            pan += data.buttons[
                self.head_up_button] * self.head_step
            pan += data.buttons[
                self.head_down_button] * -self.head_step
            if pan < self.min_pan_head:
                pan = self.min_pan_head
            elif pan > self.max_pan_head:
                pan = self.max_pan_head

        tilt = self.head_position[1]
        if data.buttons[self.deadman_button_1] or data.buttons[self.deadman_button_2]:
            tilt += data.buttons[
                self.head_up_button] * self.head_step
            tilt += data.buttons[
                self.head_down_button] * -self.head_step
            if tilt < self.min_tilt_head:
                tilt = self.min_tilt_head
            elif tilt > self.max_tilt_head:
                tilt = self.max_tilt_head

        self.head_position = [pan, tilt]

        if self.head_position != self.last_head_position:
            cmd = JointTrajectory()
            cmd.joint_names = ['head_pan_joint', 'head_tilt_joint']
            jtp = JointTrajectoryPoint()
            jtp.positions.extend(self.head_position)
            jtp.time_from_start = rospy.Duration(self.head_timestep)
            cmd.points.append(jtp)
            self.head_pub.publish(cmd)
            rospy.loginfo("head going to: " + str(cmd))
            self.last_head_position = self.head_position


class JoystickTorso(object):
    def __init__(self, config_dict=None):
        if config_dict is None:
            self.torso_command_topic = rospy.get_param('~torso_command_topic',
                                                       '/torso_controller/command')
            self.joystick_topic = rospy.get_param('~joystick_topic',
                                                  '/joy')
            self.deadman_button_1 = rospy.get_param('~deadman_button_1', 11)
            self.deadman_button_2 = rospy.get_param('~deadman_button_2', 9)
            self.torso_up_button = rospy.get_param('~torso_up_button', 12)
            self.torso_down_button = rospy.get_param('~torso_down_button', 14)
            self.min_torso = rospy.get_param('~min_torso', 0.01)
            self.max_torso = rospy.get_param('~max_torso', 0.30)
            self.torso_step = rospy.get_param('~torso_step', 0.03)
            self.torso_timestep = rospy.get_param('~torso_timestep', 0.5)
            self.node_rate_hz = rospy.get_param('~node_rate_hz', 4)
        else:
            self.torso_command_topic = config_dict['torso_command_topic']
            self.joystick_topic = config_dict['joystick_topic']
            self.deadman_button_1 = config_dict['deadman_button_1']
            self.deadman_button_2 = config_dict['deadman_button_2']
            self.torso_up_button = config_dict['torso_up_button']
            self.torso_down_button = config_dict['torso_down_button']
            self.torso_step = config_dict['torso_step']
            self.torso_timestep = config_dict['torso_timestep']
            self.min_torso = config_dict['min_torso']
            self.max_torso = config_dict['max_torso']
            self.node_rate_hz = config_dict['node_rate_hz']

        # Start with current torso position
        rospy.loginfo("Waiting for first /joint_states message...")
        self.last_js = None
        self.js_sub = rospy.Subscriber('/joint_states', JointState,
                                       self.js_cb, queue_size=1)
        while not rospy.is_shutdown() and self.last_js is None:
            rospy.sleep(0.1)

        self.torso_idx = self.last_js.name.index('torso_lift_joint')
        self.torso_position = self.get_current_torso_position()
        self.last_torso_position = self.torso_position
        self.torso_pub = rospy.Publisher(self.torso_command_topic,
                                         JointTrajectory,
                                         queue_size=1)
        rospy.loginfo("Publishing to: " + str(self.torso_pub.resolved_name))

        self.last_joy_stamp = time.time()
        self.joy_sub = rospy.Subscriber(self.joystick_topic,
                                        Joy,
                                        self.joy_cb,
                                        queue_size=1)
        rospy.loginfo("Subscribing to:" + str(self.joy_sub.resolved_name))

    def js_cb(self, data):
        self.last_js = data

    def get_current_torso_position(self):
        return self.last_js.position[self.torso_idx]

    def joy_cb(self, data):
        """
        :type data: Joy
        """
        new_t = time.time()
        last_t = self.last_joy_stamp
        if (new_t - last_t) < (1.0 / self.node_rate_hz):
            return

        self.torso_position = self.get_current_torso_position()
        if data.buttons[self.deadman_button_1] or data.buttons[self.deadman_button_2]:
            self.torso_position += data.buttons[
                self.torso_up_button] * self.torso_step
            self.torso_position += data.buttons[
                self.torso_down_button] * -self.torso_step
            if self.torso_position < self.min_torso:
                self.torso_position = self.min_torso
            elif self.torso_position > self.max_torso:
                self.torso_position = self.max_torso

            if self.torso_position != self.last_torso_position:
                cmd = JointTrajectory()
                cmd.joint_names = ['torso_lift_joint']
                jtp = JointTrajectoryPoint()
                jtp.positions.append(self.torso_position)
                jtp.time_from_start = rospy.Duration(self.torso_timestep)
                cmd.points.append(jtp)
                self.torso_pub.publish(cmd)
                rospy.loginfo("Torso going to: " + str(cmd))
                self.last_torso_position = self.torso_position


class JoystickPose(object):
    def __init__(self, config_dict=None):
        if config_dict is None:
            self.initial_pose_x = rospy.get_param('~initial_pose_x', 0.5)
            self.initial_pose_y = rospy.get_param('~initial_pose_y', 0.3)
            self.initial_pose_z = rospy.get_param('~initial_pose_z', 1.0)
            self.initial_pose_roll = rospy.get_param('~initial_pose_roll', 0.0)
            self.initial_pose_pitch = rospy.get_param(
                '~initial_pose_pitch', 0.0)
            self.initial_pose_yaw = rospy.get_param('~initial_pose_yaw', 0.0)
            self.initial_pose_frame_id = rospy.get_param(
                '~initial_pose_frame_id', 'base_link')
            self.pose_stamped_topic = rospy.get_param('~pose_stamped_topic',
                                                      '/left_arm_goal_pose')
            self.use_fk = rospy.get_param('~use_fk', False)
            self.fk_link = rospy.get_param('~fk_link', 'l_wrist_roll_link')
            self.joystick_topic = rospy.get_param('~joystick_topic',
                                                  '/joy')
            self.deadman_button = rospy.get_param('~deadman_button', 11)
            self.x_axe = rospy.get_param('~x_axe', 1)
            self.y_axe = rospy.get_param('~y_axe', 0)
            self.z_axe = rospy.get_param('~z_axe', 3)
            self.cartesian_step = rospy.get_param('~cartesian_step', 0.025)

            self.roll_axe_plus = rospy.get_param('~roll_axe_plus', 7)
            self.roll_axe_minus = rospy.get_param('~roll_axe_minus', 5)
            self.pitch_axe_plus = rospy.get_param('~pitch_axe_plus', 6)
            self.pitch_axe_minus = rospy.get_param('~pitch_axe_minus', 4)
            self.yaw_button_plus = rospy.get_param('~yaw_button_plus', 0)
            self.yaw_button_minus = rospy.get_param('~yaw_button_minus', 3)
            self.rotation_step = rospy.get_param(
                '~rotation_step', 5.0)  # Degrees

            self.reset_button = rospy.get_param('~reset_button', 16)

            self.max_x = rospy.get_param('~max_x', 1.0)
            self.min_x = rospy.get_param('~min_x', 0.0)
            self.max_y = rospy.get_param('~max_y', 0.5)
            self.min_y = rospy.get_param('~min_y', -0.5)
            self.max_z = rospy.get_param('~max_z', 0.2)
            self.min_z = rospy.get_param('~min_z', 1.5)

            self.node_rate_hz = rospy.get_param('~node_rate_hz', 10)
        else:
            self.initial_pose_x = config_dict['initial_pose_x']
            self.initial_pose_y = config_dict['initial_pose_y']
            self.initial_pose_z = config_dict['initial_pose_z']
            self.initial_pose_roll = config_dict['initial_pose_roll']
            self.initial_pose_pitch = config_dict['initial_pose_pitch']
            self.initial_pose_yaw = config_dict['initial_pose_yaw']
            self.initial_pose_frame_id = config_dict['initial_pose_frame_id']
            self.pose_stamped_topic = config_dict['pose_stamped_topic']
            self.use_fk = config_dict['use_fk']
            self.fk_link = config_dict['fk_link']
            self.joystick_topic = config_dict['joystick_topic']
            self.deadman_button = config_dict['deadman_button']
            self.x_axe = config_dict['x_axe']
            self.y_axe = config_dict['y_axe']
            self.z_axe = config_dict['z_axe']
            self.cartesian_step = config_dict['cartesian_step']

            self.roll_axe_plus = config_dict['roll_axe_plus']
            self.roll_axe_minus = config_dict['roll_axe_minus']
            self.pitch_axe_plus = config_dict['pitch_axe_plus']
            self.pitch_axe_minus = config_dict['pitch_axe_minus']
            self.yaw_button_plus = config_dict['yaw_button_plus']
            self.yaw_button_minus = config_dict['yaw_button_minus']
            self.rotation_step = config_dict['rotation_step']  # Degrees

            self.reset_button = config_dict['reset_button']

            self.max_x = config_dict['max_x']
            self.min_x = config_dict['min_x']
            self.max_y = config_dict['max_y']
            self.min_y = config_dict['min_y']
            self.max_z = config_dict['max_z']
            self.min_z = config_dict['min_z']

            self.node_rate_hz = config_dict['node_rate_hz']

        ps = PoseStamped()
        ps.pose.position.x = self.initial_pose_x
        ps.pose.position.y = self.initial_pose_y
        ps.pose.position.z = self.initial_pose_z
        quat = quaternion_from_euler(self.initial_pose_roll,
                                     self.initial_pose_pitch,
                                     self.initial_pose_yaw)
        ps.pose.orientation = Quaternion(*quat)
        ps.header.frame_id = self.initial_pose_frame_id

        # Use forward kinematics for current pose?

        self.initial_pose = ps
        if self.use_fk:
            self.gfk = GetFK(self.fk_link, self.initial_pose_frame_id)
            self.joy_pose = self.gfk.get_current_fk_pose()
        else:
            self.joy_pose = deepcopy(ps)
        self.pub = rospy.Publisher(self.pose_stamped_topic,
                                   PoseStamped,
                                   queue_size=1)
        rospy.loginfo("Publishing to: " + str(self.pub.resolved_name))

        self.last_joy_stamp = time.time()
        self.last_pose = self.joy_pose.pose
        self.joy_sub = rospy.Subscriber(self.joystick_topic,
                                        Joy,
                                        self.joy_cb,
                                        queue_size=1)
        rospy.loginfo("Subscribing to:" + str(self.joy_sub.resolved_name))

        rospy.loginfo("Starting!")

    def joy_cb(self, data):
        """
        :type data: Joy
        """
        new_t = time.time()
        last_t = self.last_joy_stamp
        # Only process a message every 1.0/node_rate_hz to not spam the pose
        if (new_t - last_t) < (1.0 / self.node_rate_hz):
            return

        if data.buttons[self.deadman_button]:
            if self.use_fk:
                self.joy_pose = self.gfk.get_current_fk_pose()

            # The reset takes priority
            if data.buttons[self.reset_button]:
                self.joy_pose = deepcopy(self.initial_pose)

            self.joy_pose.pose.position.x += data.axes[
                self.x_axe] * self.cartesian_step
            if self.joy_pose.pose.position.x > self.max_x:
                self.joy_pose.pose.position.x = self.max_x
            if self.joy_pose.pose.position.x < self.min_x:
                self.joy_pose.pose.position.x = self.min_x
            self.joy_pose.pose.position.y += data.axes[
                self.y_axe] * self.cartesian_step
            if self.joy_pose.pose.position.y > self.max_y:
                self.joy_pose.pose.position.y = self.max_y
            if self.joy_pose.pose.position.y < self.min_y:
                self.joy_pose.pose.position.y = self.min_y
            self.joy_pose.pose.position.z += data.axes[
                self.z_axe] * self.cartesian_step
            if self.joy_pose.pose.position.z > self.max_z:
                self.joy_pose.pose.position.z = self.max_z
            if self.joy_pose.pose.position.z < self.min_z:
                self.joy_pose.pose.position.z = self.min_z

            roll, pitch, yaw = euler_from_quaternion(
                msg_to_list(self.joy_pose.pose.orientation))
            roll += data.axes[self.roll_axe_plus] * \
                radians(self.rotation_step)
            roll += data.axes[self.roll_axe_minus] * \
                radians(-self.rotation_step)
            pitch += data.axes[self.pitch_axe_plus] * \
                radians(self.rotation_step)
            pitch += data.axes[self.pitch_axe_minus] * \
                radians(-self.rotation_step)
            yaw += (data.buttons[self.yaw_button_plus] * radians(self.rotation_step) +
                    data.buttons[self.yaw_button_minus] * radians(-self.rotation_step))
            self.joy_pose.pose.orientation = Quaternion(
                *quaternion_from_euler(roll, pitch, yaw))

            self.last_joy_stamp = time.time()

            if not are_poses_equal(self.joy_pose.pose, self.last_pose):
                rospy.loginfo("Publishing pose: " + str(self.joy_pose.pose))
                self.pub.publish(self.joy_pose)
                self.last_pose = deepcopy(self.joy_pose.pose)


def are_poses_equal(p1, p2):
    pos1 = p1.position
    pos2 = p2.position
    if isclose(pos1.x, pos2.x) and isclose(pos1.y, pos2.y) and isclose(pos1.z, pos2.z):
        o1 = p1.orientation
        o2 = p2.orientation
        r1, p1, y1 = euler_from_quaternion([o1.x, o1.y, o1.z, o1.w])
        r2, p2, y2 = euler_from_quaternion([o2.x, o2.y, o2.z, o2.w])
        if isclose(r1, r2) and isclose(p1, p2) and isclose(y1, y2):
            return True
    return False


def isclose(n1, n2, tol=0.001):
    return abs(n1 - n2) <= tol
