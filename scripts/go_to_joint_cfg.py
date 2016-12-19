#!/usr/bin/env python

import rospy
import sys
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python_tools.go_to_configuration import GoToConfiguration

# Build a useful mapping from MoveIt error codes to error names
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

"""
Give it a list of links and its wanted positions
and it will send a trajectory to get there
using MoveIt!

You must have launched on the robot
roslaunch pr2_moveit_config move_group.launch
beforehand.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""

if __name__ == '__main__':
    argv = sys.argv
    if "--help" in argv or "-h" in argv or len(argv) < 4:
        print "Usage:"
        print argv[0] + " joint1 joint2 ... jointN pos1 pos2 ... posN time(seconds)"
        print
        print "Example:"
        print argv[0] + "head_pan_joint 0.1 1.0"
        exit(0)

    print "Connecting to ROS node..."
    rospy.init_node('go_to_pose')
    gtp = GoToConfiguration()

    time = float(argv[-1])
    joints = []
    positions = []
    # The joint + positions should be even
    j_and_p = argv[1:-1]
    print "j_and_p: " + str(j_and_p)
    if not len(j_and_p) % 2 == 0:
        rospy.logerr("Command malformed... use " + argv[0] + " -h")
        exit(0)
    # first half should be joint names
    joints = j_and_p[:len(j_and_p) / 2]
    print "joints: " + str(joints)
    positions = j_and_p[len(j_and_p) / 2:]
    positions = [float(p) for p in positions]
    print "positions: " + str(positions)

    resp = gtp.go_to_configuration(joints, positions, time)
    rospy.loginfo(str(resp))
    err_code = resp.error_code.val
    rospy.loginfo("Error is: " + moveit_error_dict[err_code])
