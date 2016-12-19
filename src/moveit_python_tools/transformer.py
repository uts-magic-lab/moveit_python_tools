#!/usr/bin/env python

import rospy
import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, PointStamped, QuaternionStamped
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import Vector3, Vector3Stamped
from math import radians

"""
Class to interact with TF.

Author: Sammy Pfeiffer
"""


class Transformer(object):
    def __init__(self):
        """
        Class to make easier the use of TF.
        """
        self.tf_l = tf.TransformListener()
        # We need to leave a bit of time for the subscriber
        # to TF topic inside of TransformListener
        # to get some data
        rospy.sleep(0.5)

    def transform(self, thing, from_frame, to_frame, type_data=None):
        """
        Given a thing to transform, deduct by type how to transform it.

        :param thing: can be of types: PoseStamped, Pose, PointStamped, Point,
            QuaternionStamped, Quaternion,
            Vector3 (with type_data: xyz, rpydeg, rpyrad),
            list or tuple (with type_data: xyz, rpydeg, rpyrad, quat)
        """
        if isinstance(thing, PoseStamped):
            return self.transform_pose(thing.pose, from_frame, to_frame)

        elif isinstance(thing, Pose):
            return self.transform_pose(thing, from_frame, to_frame)

        elif isinstance(thing, PointStamped):
            return self.transform_point(thing.point, from_frame, to_frame)

        elif isinstance(thing, Point):
            return self.transform_point(thing, from_frame, to_frame)

        elif isinstance(thing, QuaternionStamped):
            return self.transform_quaternion(thing.quaternion, from_frame, to_frame)

        elif isinstance(thing, Quaternion):
            return self.transform_quaternion(thing, from_frame, to_frame)

        elif isinstance(thing, Vector3) or isinstance(thing, Vector3Stamped):
            if isinstance(thing, Vector3Stamped):
                thing = thing.vector
            if type_data is "xyz":
                point = Point(thing.x, thing.y, thing.z)
                return self.transform_point(point, from_frame, to_frame)
            elif type_data is "rpydeg":
                r, p, y = radians(thing.x), radians(thing.y), radians(thing.z)
                quat = quaternion_from_euler(r, p, y)
                quaternion = Quaternion(*quat)
                return self.transform_quaternion(quaternion, from_frame, to_frame)
            elif type_data is "rpyrad":
                quat = quaternion_from_euler(thing.x, thing.y, thing.z)
                quaternion = Quaternion(*quat)
                return self.transform_quaternion(quaternion, from_frame, to_frame)
            elif type_data is None:
                rospy.logerr("Didn't specify what type_data this list is.")
                rospy.logerr("Don't know what to do with: " + str(thing))
            else:
                rospy.logerr("I don't know what type_data: " + str(type_data) +
                             " is. Try: xyz, rpydeg, rpyrad")
                rospy.logerr("Don't know what to do with: " + str(thing))
            return None

        elif isinstance(thing, list) or isinstance(thing, tuple):
            if type_data is "xyz" and len(thing) == 3:
                point = Point(thing[0], thing[1], thing[2])
                return self.transform_point(point, from_frame, to_frame)
            elif type_data is "rpydeg" and len(thing) == 3:
                r, p, y = radians(thing[0]), radians(
                    thing[1]), radians(thing[2])
                quat = quaternion_from_euler(r, p, y)
                quaternion = Quaternion(*quat)
                return self.transform_quaternion(quaternion, from_frame, to_frame)
            elif type_data is "rpyrad" and len(thing) == 3:
                quat = quaternion_from_euler(thing[0], thing[1], thing[2])
                quaternion = Quaternion(*quat)
                return self.transform_quaternion(quaternion, from_frame, to_frame)
            elif type_data is "quat" and len(thing) == 4:
                quaternion = Quaternion(thing[0], thing[1], thing[2], thing[3])
                return self.transform_quaternion(quaternion, from_frame, to_frame)
            elif type_data is None:
                rospy.logerr("Didn't specify what type_data this list is.")
                rospy.logerr("Don't know what to do with: " + str(thing))
            else:
                rospy.logerr("I don't know what type_data: " + str(type_data) +
                             " is. Try: xyz, rpydeg, rpyrad, quat")
                rospy.logerr("Don't know what to do with: " + str(thing))
            return None

    def transform_pose(self, pose, from_frame, to_frame):
        """
        Transform the 'pose' from frame 'from_frame'
         to frame 'to_frame'

        :param geometry_msgs/Pose pose: 3D Pose to transform.
        :param str from_frame: frame that the pose belongs to.
        :param str to_frame: to what frame transform.
        """
        ps = PoseStamped()
        # ps.header.stamp = #self.tf_l.getLatestCommonTime(from_frame,
        # to_frame)
        ps.header.frame_id = from_frame
        ps.pose = pose
        transform_ok = False
        while not transform_ok and not rospy.is_shutdown():
            try:
                target_ps = self.tf_l.transformPose(to_frame, ps)
                transform_ok = True
            except tf.ExtrapolationException as e:
                rospy.logwarn(
                    "Exception on transforming pose... trying again \n(" +
                    str(e) + ")")
                rospy.sleep(0.2)
                ps.header.stamp = self.tf_l.getLatestCommonTime(
                    from_frame, to_frame)
            except tf.LookupException as e:
                rospy.logwarn(
                    "Exception on transforming pose... trying again \n(" +
                    str(e) + ")")
                rospy.sleep(0.2)

        return target_ps

    def transform_point(self, point, from_frame, to_frame):
        """
        Transform the Point 'point' from frame 'from_frame'
         to frame 'to_frame'

        :param geometry_msgs/Point pose: 3D Point to transform.
        :param str from_frame: frame that the pose belongs to.
        :param str to_frame: to what frame transform.
        """
        ps = PointStamped()
        # ps.header.stamp = #self.tf_l.getLatestCommonTime(from_frame,
        # to_frame)
        ps.header.frame_id = from_frame
        ps.point = point
        transform_ok = False
        while not transform_ok and not rospy.is_shutdown():
            try:
                target_ps = self.tf_l.transformPoint(to_frame, ps)
                transform_ok = True
            except tf.ExtrapolationException as e:
                rospy.logwarn(
                    "Exception on transforming point... trying again \n(" +
                    str(e) + ")")
                rospy.sleep(0.2)
                ps.header.stamp = self.tf_l.getLatestCommonTime(
                    from_frame, to_frame)
            except tf.LookupException as e:
                rospy.logwarn(
                    "Exception on transforming point... trying again \n(" +
                    str(e) + ")")
                rospy.sleep(0.2)

        return target_ps

    def transform_quaternion(self, quaternion, from_frame, to_frame):
        """
        Transform the Quaternion 'quaternion' from frame 'from_frame'
         to frame 'to_frame'

        :param geometry_msgs/Quaternion pose: quaternion to transform.
        :param str from_frame: frame that the pose belongs to.
        :param str to_frame: to what frame transform.
        """
        qs = QuaternionStamped()
        # ps.header.stamp = #self.tf_l.getLatestCommonTime(from_frame,
        # to_frame)
        qs.header.frame_id = from_frame
        qs.quaternion = quaternion
        transform_ok = False
        while not transform_ok and not rospy.is_shutdown():
            try:
                target_qs = self.tf_l.transformQuaternion(to_frame, qs)
                transform_ok = True
            except tf.ExtrapolationException as e:
                rospy.logwarn(
                    "Exception on transforming quaternion... trying again \n(" +
                    str(e) + ")")
                rospy.sleep(0.2)
                qs.header.stamp = self.tf_l.getLatestCommonTime(
                    from_frame, to_frame)
            except tf.LookupException as e:
                rospy.logwarn(
                    "Exception on transforming quaternion... trying again \n(" +
                    str(e) + ")")
                rospy.sleep(0.2)

        return target_qs
