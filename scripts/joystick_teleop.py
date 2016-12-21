#! /usr/bin/env python

"""
Teleop PR2 using a PS3 controller.

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

By instantiating all the classes in the same script
they will share the subscribers to the common topics.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""

import rospy
from moveit_python_tools.joystick_teleop import JoystickPose
from moveit_python_tools.joystick_teleop import JoystickGripper
from moveit_python_tools.joystick_teleop import JoystickTorso

if __name__ == '__main__':
    rospy.init_node('joystick_pose_pub')
    cfg_d_lp = {'initial_pose_x': 0.5,
                'initial_pose_y': 0.2,
                'initial_pose_z': 1.0,
                'initial_pose_roll': 0.0,
                'initial_pose_pitch': 0.0,
                'initial_pose_yaw': 0.0,
                'initial_pose_frame_id': 'base_link',
                'pose_stamped_topic': '/left_arm_goal_pose',
                'use_fk': True,
                'fk_link': 'l_wrist_roll_link',
                'joystick_topic': '/joy',
                'deadman_button': 11,
                'x_axe': 1,
                'y_axe': 0,
                'z_axe': 3,
                'cartesian_step': 0.035,

                'roll_axe_plus': 7,
                'roll_axe_minus': 5,
                'pitch_axe_plus': 6,
                'pitch_axe_minus': 4,
                'yaw_button_plus': 0,
                'yaw_button_minus': 3,
                'rotation_step': 9.0,  # Degrees

                'reset_button': 16,

                'max_x': 0.8,
                'min_x': 0.0,
                'max_y': 0.75,
                'min_y': -0.5,
                'max_z': 1.5,
                'min_z': 0.0,

                'node_rate_hz': 10}
    jp_left = JoystickPose(config_dict=cfg_d_lp)
    cfg_d_rp = {'initial_pose_x': 0.5,
                'initial_pose_y': -0.2,
                'initial_pose_z': 1.0,
                'initial_pose_roll': 0.0,
                'initial_pose_pitch': 0.0,
                'initial_pose_yaw': 0.0,
                'initial_pose_frame_id': 'base_link',
                'pose_stamped_topic': '/right_arm_goal_pose',
                'use_fk': True,
                'fk_link': 'r_wrist_roll_link',
                'joystick_topic': '/joy',
                'deadman_button': 9,
                'x_axe': 1,
                'y_axe': 0,
                'z_axe': 3,
                'cartesian_step': 0.035,

                'roll_axe_plus': 7,
                'roll_axe_minus': 5,
                'pitch_axe_plus': 6,
                'pitch_axe_minus': 4,
                'yaw_button_plus': 0,
                'yaw_button_minus': 3,
                'rotation_step': 9.0,  # Degrees

                'reset_button': 16,

                'max_x': 0.8,
                'min_x': 0.0,
                'max_y': 0.5,
                'min_y': -0.75,
                'max_z': 1.5,
                'min_z': 0.0,

                'node_rate_hz': 10}
    jp_right = JoystickPose(config_dict=cfg_d_rp)

    cfg_d_lg = {
        'gripper_command_topic': '/l_gripper_controller/command',
        'gripper_joint_name': 'l_gripper_joint',
        'joystick_topic': '/joy',
        'deadman_button': 11,
        'open_button': 13,
        'close_button': 15,
        'gripper_step': 0.005,
        'min_gripper_pos': 0.00,
        'max_gripper_pos': 0.1,
        'node_rate_hz': 4.0
    }
    cfg_d_rg = {
        'gripper_command_topic': '/r_gripper_controller/command',
        'gripper_joint_name': 'r_gripper_joint',
        'joystick_topic': '/joy',
        'deadman_button': 9,
        'open_button': 13,
        'close_button': 15,
        'gripper_step': 0.005,
        'min_gripper_pos': 0.00,
        'max_gripper_pos': 0.1,
        'node_rate_hz': 4.0
    }
    gripper_left = JoystickGripper(config_dict=cfg_d_lg)
    gripper_right = JoystickGripper(config_dict=cfg_d_rg)

    cfg_d_t = {
        'torso_command_topic': '/torso_controller/command',
        'joystick_topic': '/joy',
        'deadman_button_1': 11,
        'deadman_button_2': 9,
        'torso_up_button': 12,
        'torso_down_button': 14,
        'torso_step': 0.005,
        'torso_timestep': 0.5,
        'min_torso': 0.01,
        'max_torso': 0.30,
        'node_rate_hz': 4.0
    }
    torso = JoystickTorso(config_dict=cfg_d_t)

    rospy.spin()
