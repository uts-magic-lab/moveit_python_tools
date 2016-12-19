#!/usr/bin/env python

import rospy
from moveit_python_tools.get_ik import GetIK
from moveit_python_tools.go_to_configuration import GoToConfiguration
import sys
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
from math import radians

from moveit_msgs.msg import MoveItErrorCodes
# Build a useful mapping from MoveIt error codes to error names
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

"""
Give it a pose stamped
and the robot will get and IK and execute a
trajectory to it.

Fun to explore where it can get.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


if __name__ == '__main__':
    argv = sys.argv
    if len(sys.argv) not in [9, 10, 11]:
        print "Usage:"
        print argv[0] + " x y z roll pitch yaw frame_id time (-rad)"  # 8, 9
        print argv[0] + " x y z qx qy qz qw frame_id time (-rad)"  # 9, 10
        exit(0)

    print "Connecting to ROS node..."
    rospy.init_node('get_ik_and_go')
    gik = GetIK()
    gtp = GoToConfiguration()

    use_radians = False
    if sys.argv[-1] == "-rad":
        use_radians = True
        argv = argv[:-1]

    time = float(argv[-1])
    argv = argv[:-1]

    if len(argv) == 8 or len(argv) == 9:
        ps = PoseStamped()
        ps.pose.position.x = float(argv[1])
        ps.pose.position.y = float(argv[2])
        ps.pose.position.z = float(argv[3])

        if len(argv) == 8:
            roll = float(argv[4])
            pitch = float(argv[5])
            yaw = float(argv[6])
            if not use_radians:
                roll = radians(roll)
                pitch = radians(pitch)
                yaw = radians(yaw)
            q = quaternion_from_euler(roll, pitch, yaw)
            quat = Quaternion(*q)
            ps.pose.orientation = quat
            from_frame = argv[7]
        else:
            ps.pose.orientation.x = float(argv[4])
            ps.pose.orientation.y = float(argv[5])
            ps.pose.orientation.z = float(argv[6])
            ps.pose.orientation.w = float(argv[7])
            from_frame = argv[8]

        ps.header.frame_id = from_frame
        rospy.loginfo("Getting IK!")
        resp = gik.get_ik(ps, group='right_arm', duration=3.0,
                          attempts=0)
        rospy.loginfo(str(resp))
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

            gtp.go_to_pose(joint_names, positions, time)
            rospy.loginfo(str(resp))
            err_code = resp.error_code.val
            rospy.loginfo("Error is: " + moveit_error_dict[err_code])
