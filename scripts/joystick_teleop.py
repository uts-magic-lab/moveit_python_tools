#! /usr/bin/env python

"""
Multiple classes to teleop using a PS3 controller.

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
from control_msgs.msg import JointTrajectoryControllerState
from pr2_controllers_msgs.msg import Pr2GripperCommand
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
            self.joystick_topic = config_dict['joystick_topic']
            self.deadman_button = config_dict['deadman_button']
            self.open_button = config_dict['open_button']
            self.close_button = config_dict['close_button']
            self.gripper_step = config_dict['gripper_step']
            self.min_gripper_pos = config_dict['min_gripper_pos']
            self.max_gripper_pos = config_dict['max_gripper_pos']
            self.node_rate_hz = config_dict['node_rate_hz']

        self.gripper_amount = 0.0
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

    def joy_cb(self, data):
        """
        :type data: Joy
        """
        new_t = time.time()
        last_t = self.last_joy_stamp
        if (new_t - last_t) < (1.0 / self.node_rate_hz):
            return

        if data.buttons[self.deadman_button]:
            self.gripper_amount += data.buttons[
                self.open_button] * self.gripper_amount
            self.gripper_amount += data.buttons[-self.close_button] * \
                self.gripper_amount
            if self.gripper_amount < self.min_gripper_pos:
                self.gripper_amount = self.min_gripper_pos
            elif self.gripper_amount > self.max_gripper_pos:
                self.gripper_amount = self.max_gripper_pos

            cmd = Pr2GripperCommand()
            cmd.max_effort = 100.0
            cmd.position = self.gripper_amount
            self.gripper_pub.publish(cmd)


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
        rospy.loginfo("Waiting for one /joint_states message...")
        js_msg = rospy.wait_for_message('/joint_states', JointState)
        t_idx = js_msg.name.index('torso_lift_joint')
        self.torso_position = js_msg.position[t_idx]
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

    def joy_cb(self, data):
        """
        :type data: Joy
        """
        new_t = time.time()
        last_t = self.last_joy_stamp
        if (new_t - last_t) < (1.0 / self.node_rate_hz):
            return

        if data.buttons[self.deadman_button_1] or data.buttons[self.deadman_button_2]:
            self.torso_position += data.buttons[
                self.torso_up_button] * self.torso_step
            self.torso_position += data.buttons[-self.torso_down_button] * \
                self.torso_step
            if self.torso_position < self.min_torso:
                self.torso_position = self.min_torso
            elif self.torso_position > self.max_torso:
                self.torso_position = self.max_torso

            cmd = JointTrajectory()
            cmd.joint_names = ['torso_lift_joint']
            jtp = JointTrajectoryPoint()
            jtp.positions.append(self.torso_position)
            jtp.time_from_start = rospy.Duration(self.torso_timestep)
            cmd.points.append(jtp)
            self.self.torso_pub.publish(cmd)


class JoystickPose(object):
    def __init__(self, config_dict=None):
        if config_dict is None
            self.initial_pose_x = rospy.get_param('~initial_pose_x', 0.5)
            self.initial_pose_y = rospy.get_param('~initial_pose_y', 0.3)
            self.initial_pose_z = rospy.get_param('~initial_pose_z', 1.0)
            self.initial_pose_roll = rospy.get_param('~initial_pose_roll', 0.0)
            self.initial_pose_pitch = rospy.get_param('~initial_pose_pitch', 0.0)
            self.initial_pose_yaw = rospy.get_param('~initial_pose_yaw', 0.0)
            self.initial_pose_frame_id = rospy.get_param(
                '~initial_pose_frame_id', 'base_link')
            self.pose_stamped_topic = rospy.get_param('~pose_stamped_topic',
                                                      '/left_arm_goal_pose')
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
            self.yaw_button_plus = rospy.get_param('~yaw_axe_plus', 0)
            self.yaw_button_minus = rospy.get_param('~yaw_axe_plus', 3)
            self.rotation_step = rospy.get_param('~rotation_step', 5.0)  # Degrees

            self.reset_button = rospy.get_param('~reset_button', 16)

            self.max_x = rospy.get_param('~max_x', 1.0)
            self.min_x = rospy.get_param('~min_x', 0.0)
            self.max_y = rospy.get_param('~max_y', 0.5)
            self.min_y = rospy.get_param('~min_y', -0.5)
            self.max_z = rospy.get_param('~max_z', 0.2)
            self.min_z = rospy.get_param('~min_z', 1.5)

            self.node_rate_hz = rospy.get_param('~node_rate_hz', 10)
        else:
            self.initial_pose_x = config['initial_pose_x']
            self.initial_pose_y = config['initial_pose_y']
            self.initial_pose_z = config['initial_pose_z']
            self.initial_pose_roll = config['initial_pose_roll']
            self.initial_pose_pitch = config['initial_pose_pitch']
            self.initial_pose_yaw = config['initial_pose_yaw']
            self.initial_pose_frame_id = config['initial_pose_frame_id']
            self.pose_stamped_topic = config['pose_stamped_topic']
            self.joystick_topic = config['joystick_topic']
            self.deadman_button = config['deadman_button']
            self.x_axe = config['x_axe']
            self.y_axe = config['y_axe']
            self.z_axe = config['z_axe']
            self.cartesian_step = config['cartesian_step']

            self.roll_axe_plus = config['roll_axe_plus']
            self.roll_axe_minus = config['roll_axe_minus']
            self.pitch_axe_plus = config['pitch_axe_plus']
            self.pitch_axe_minus = config['pitch_axe_minus']
            self.yaw_button_plus = config['yaw_axe_plus']
            self.yaw_button_minus = config['yaw_axe_plus']
            self.rotation_step = config['rotation_step']  # Degrees

            self.reset_button = config['reset_button']

            self.max_x = config['max_x']
            self.min_x = config['min_x']
            self.max_y = config['max_y']
            self.min_y = config['min_y']
            self.max_z = config['max_z']
            self.min_z = config['min_z']

            self.node_rate_hz = config['node_rate_hz']

        ps = PoseStamped()
        ps.pose.position.x = self.initial_pose_x
        ps.pose.position.y = self.initial_pose_y
        ps.pose.position.z = self.initial_pose_z
        quat = quaternion_from_euler(self.initial_pose_roll,
                                     self.initial_pose_pitch,
                                     self.initial_pose_yaw)
        ps.pose.orientation = Quaternion(*quat)
        ps.header.frame_id = self.initial_pose_frame_id

        self.initial_pose = ps
        self.joy_pose = deepcopy(ps)
        self.pub = rospy.Publisher(self.pose_stamped_topic, PoseStamped)
        rospy.loginfo("Publishing to: " + str(self.pub.resolved_name))
        self.deadman_pushed = False

        self.last_joy_stamp = time.time()
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
            if data.buttons[self.reset_button]:
                self.joy_pose = deepcopy(self.initial_pose)
                return

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
                radians(-self.rotation_step)
            roll += data.axes[self.roll_axe_minus] * \
                radians(self.rotation_step)
            pitch += data.axes[self.pitch_axe_plus] * \
                radians(-self.rotation_step)
            pitch += data.axes[self.pitch_axe_minus] * \
                radians(self.rotation_step)
            yaw += (data.buttons[self.yaw_button_plus] * radians(self.rotation_step) +
                    data.buttons[self.yaw_button_minus] * radians(-self.rotation_step))
            self.joy_pose.pose.orientation = Quaternion(
                *quaternion_from_euler(roll, pitch, yaw))

            self.last_joy_stamp = time.time()

            rospy.loginfo("Publishing pose: " + str(self.joy_pose.pose))
            self.pub.publish(self.joy_pose)


if __name__ == '__main__':
    rospy.init_node('joystick_pose_pub')
    cfg_d_lp = {
            'initial_pose_x': 0.5,
            'initial_pose_y': 0.3,
            'initial_pose_z': 1.0,
            'initial_pose_roll': 0.0,
            'initial_pose_pitch': 0.0,
            'initial_pose_yaw': 0.0,
            'initial_pose_frame_id': 'base_link',
            'pose_stamped_topic': '/left_arm_goal_pose',
            'joystick_topic': '/joy',
            'deadman_button': 11,
            'x_axe': 1,
            'y_axe': 0,
            'z_axe': 3,
            'cartesian_step': 0.025,

            'roll_axe_plus': 7,
            'roll_axe_minus': 5,
            'pitch_axe_plus': 6,
            'pitch_axe_minus': 4,
            'yaw_axe_plus': 0,
            'yaw_axe_plus': 3,
            'rotation_step': 5.0,  # Degrees

            'reset_button': 16,

            'max_x': 0.7,
            'min_x': 0.0,
            'max_y': 0.5,
            'min_y': -0.5,
            'max_z': 1.5,
            'min_z': 0.2,

            'node_rate_hz': 10
    }
    jp_left = JoystickPose(config_dict=cfg_d_lp)
    cfg_d_rp = {
            'initial_pose_x': 0.5,
            'initial_pose_y': -0.3,
            'initial_pose_z': 1.0,
            'initial_pose_roll': 0.0,
            'initial_pose_pitch': 0.0,
            'initial_pose_yaw': 0.0,
            'initial_pose_frame_id': 'base_link',
            'pose_stamped_topic': '/right_arm_goal_pose',
            'joystick_topic': '/joy',
            'deadman_button': 9,
            'x_axe': 1,
            'y_axe': 0,
            'z_axe': 3,
            'cartesian_step': 0.025,

            'roll_axe_plus': 7,
            'roll_axe_minus': 5,
            'pitch_axe_plus': 6,
            'pitch_axe_minus': 4,
            'yaw_axe_plus': 0,
            'yaw_axe_plus': 3,
            'rotation_step': 5.0,  # Degrees

            'reset_button': 16,

            'max_x': 0.7,
            'min_x': 0.0,
            'max_y': 0.5,
            'min_y': -0.5,
            'max_z': 1.5,
            'min_z': 0.2,

            'node_rate_hz': 10
    }
    jp_right = JoystickPose()

    cfg_d_lg = {
        'gripper_command_topic': '/l_gripper_controller/command',
        'joystick_topic': '/joy',
        'deadman_button': 11,
        'open_button': 13,
        'close_button': 15,
        'gripper_step': 0.11,
        'min_gripper_pos': 0.08,
        'max_gripper_pos': 1.0,
        'node_rate_hz': 4.0
    }
    cfg_d_rg = {
        'gripper_command_topic': '/r_gripper_controller/command',
        'joystick_topic': '/joy',
        'deadman_button': 9,
        'open_button': 13,
        'close_button': 15,
        'gripper_step': 0.11,
        'min_gripper_pos': 0.08,
        'max_gripper_pos': 1.0,
        'node_rate_hz': 4.0
    }
    gripper_left = JoystickGripper(config_dict=cfg_d_lg)
    gripper_right = JoystickGripper(config_dict=cfg_d_rg)

    cfg_d_t = {
        'torso_command_topic': '/torso_controller/command',
        'joystick_topic': '/joy',
        'deadman_button_1': 11,
        'deadman_button_2': 9,
        'torso_up_button': 13,
        'torso_down_button': 15,
        'torso_step': 0.11,
        'torso_timestep': 0.11,
        'min_torso': 0.08,
        'max_torso': 1.0,
        'node_rate_hz': 4.0
    }
    torso = JoystickTorso(config_dict=cfg_d_t)

    rospy.spin()
