#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
import sys
from math import radians
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python_tools.get_ik import GetIK

# Build a useful mapping from MoveIt error codes to error names
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

"""
Give a pose stamped pose
and it will get the corresponding IK.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


if __name__ == '__main__':
    argv = sys.argv
    if len(sys.argv) not in [8, 9, 10]:
        print "Usage:"
        print argv[0] + " x y z roll pitch yaw frame_id (-rad)"  # 8, 9
        print argv[0] + " x y z qx qy qz qw frame_id (-rad)"  # 9, 10
        exit(0)

    print "Connecting to ROS node..."
    rospy.init_node('get_iker')
    gik = GetIK(group='right_arm_torso')

    use_radians = False
    if sys.argv[-1] == "-rad":
        use_radians = True
        argv = argv[:-1]

    # Asking for a pose transformation
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
        resp = gik.get_ik(ps)
        rospy.loginfo(str(resp))
        err_code = resp.error_code.val
        rospy.loginfo("Error is: " + moveit_error_dict[err_code])
