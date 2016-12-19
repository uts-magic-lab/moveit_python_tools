#!/usr/bin/env python

import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseStamped, PointStamped
from geometry_msgs.msg import Quaternion, QuaternionStamped
import sys
from math import radians, degrees
from moveit_python_tools.transformer import Transformer

"""
Useful command line tool to transform
pose
point
quaternion

from one frame to another.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


if __name__ == '__main__':
    argv = sys.argv
    if len(argv) not in [7, 8, 10, 11]:
        print "Usage:"
        print argv[0] + " [pose/point/orientation]"
        print argv[0] + " pose x y z roll pitch yaw from_frame to_frame (-rad)"  # 10
        print argv[0] + " pose x y z qx qy qz qw from_frame to_frame"  # 11
        print argv[0] + " point x y z from_frame to_frame"  # 7
        print argv[0] + " orientation roll pitch yaw from_frame to_frame (-rad)"  # 7
        print argv[0] + " orientation qx qy qz qw from_frame to_frame"  # 8
        print
        print "For example:"
        print argv[0] + " pose 0.0 0.0 0.0 0.0 0.0 0.0 base_laser_link base_link"
        print "Use -rad if you want to use roll pitch yaw in radians instead of degrees"
        exit(0)

    print "Initializing ROS node..."
    rospy.init_node('tf_transformer', anonymous=True)
    # argv = rospy.myargv()
    tfer = Transformer()
    use_radians = False
    if argv[-1] == "-rad":
        use_radians = True
        argv = argv[:-1]

    # Asking for a pose transformation
    if len(argv) == 10 or len(argv) == 11:
        ps = PoseStamped()
        ps.pose.position.x = float(argv[2])
        ps.pose.position.y = float(argv[3])
        ps.pose.position.z = float(argv[4])

        if len(argv) == 10:
            roll = float(argv[5])
            pitch = float(argv[6])
            yaw = float(argv[7])
            if not use_radians:
                roll = radians(roll)
                pitch = radians(pitch)
                yaw = radians(yaw)
            q = quaternion_from_euler(roll, pitch, yaw)
            quat = Quaternion(*q)
            ps.pose.orientation = quat
            from_frame = argv[8]
            to_frame = argv[9]
        else:
            ps.pose.orientation.x = float(argv[5])
            ps.pose.orientation.y = float(argv[6])
            ps.pose.orientation.z = float(argv[7])
            ps.pose.orientation.w = float(argv[8])
            from_frame = argv[9]
            to_frame = argv[10]

        ps.header.frame_id = from_frame
        rospy.loginfo("Transforming " + str(ps))

        tf_ps = tfer.transform_pose(ps, from_frame, to_frame)
        rospy.loginfo("to")
        rospy.loginfo(str(tf_ps))
        p = tf_ps.pose.position
        o = tf_ps.pose.orientation
        rospy.loginfo("In frame: " + to_frame)
        rospy.loginfo("[px, py, pz], [ox, oy, oz, ow]: " + str(([p.x, p.y, p.z], [o.x, o.y, o.z, o.w])))
        r, pitch, y = euler_from_quaternion([o.x, o.y, o.z, o.w])
        rospy.loginfo("[px, py, pz], [r, p, y](radians): " + str(([p.x, p.y, p.z], [r, pitch, y])))
        rospy.loginfo("[px, py, pz], [r, p, y](degrees): " + str(([p.x, p.y, p.z], [degrees(r), degrees(pitch), degrees(y)])))

    # point
    elif len(argv) == 7 and argv[1] == 'point':
        ps = PointStamped()
        ps.point.x = float(argv[2])
        ps.point.y = float(argv[3])
        ps.point.z = float(argv[4])
        from_frame = argv[5]
        to_frame = argv[6]

        ps.header.frame_id = from_frame
        rospy.loginfo("Transforming " + str(ps))
        tf_ps = tfer.transform_point(ps, from_frame, to_frame)
        rospy.loginfo("to")
        rospy.loginfo(str(tf_ps))
        p = tf_ps.point
        rospy.loginfo("In frame: " + to_frame)
        rospy.loginfo("[px, py, pz]: " + str([p.x, p.y, p.z]))

    # orientation
    elif len(argv) == 7 or len(argv) == 8:
        qs = QuaternionStamped()
        if len(argv) == 8:
            qs.quaternion.x = float(argv[2])
            qs.quaternion.y = float(argv[3])
            qs.quaternion.z = float(argv[4])
            qs.quaternion.w = float(argv[5])
            from_frame = argv[6]
            to_frame = argv[7]

        elif len(argv) == 7:
            roll = float(argv[2])
            pitch = float(argv[3])
            yaw = float(argv[4])
            if not use_radians:
                roll = radians(roll)
                pitch = radians(pitch)
                yaw = radians(yaw)
            q = quaternion_from_euler(roll, pitch, yaw)
            quat = Quaternion(*q)
            qs.quaternion = quat

            from_frame = argv[5]
            to_frame = argv[6]

        qs.header.frame_id = from_frame
        rospy.loginfo("Transforming " + str(qs))
        tf_qs = tfer.transform_quaternion(qs, from_frame, to_frame)
        rospy.loginfo("to")
        rospy.loginfo(str(tf_qs))
        o = tf_qs.quaternion
        r, p, y = euler_from_quaternion([o.x, o.y, o.z, o.w])
        rospy.loginfo("In frame: " + to_frame)
        rospy.loginfo("[r, p, y](radians): " + str(([r, p, y])))
        rospy.loginfo("[r, p, y](degrees): " + str(([degrees(r), degrees(p), degrees(y)])))
