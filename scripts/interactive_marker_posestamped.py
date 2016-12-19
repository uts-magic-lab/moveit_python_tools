#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarkerControl, Marker, InteractiveMarker, \
    InteractiveMarkerFeedback, InteractiveMarkerUpdate, InteractiveMarkerPose, MenuEntry
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler

"""
Created on 05/09/15.
Updated on 19/12/16.

Interactive marker that publishes a PoseStamped message.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""
# based on https://github.com/ros-visualization/visualization_tutorials/blob/indigo-devel/interactive_marker_tutorials/scripts/basic_controls.py
# and
# https://github.com/ros-visualization/visualization_tutorials/blob/indigo-devel/interactive_marker_tutorials/scripts/menu.py


class InteractiveMarkerPoseStampedPublisher():

    def __init__(self, topic,
                 initial_x, initial_y, initial_z,
                 initial_roll, initial_pitch, initial_yaw,
                 frame_id):
        self.server = InteractiveMarkerServer("posestamped_im")
        self.menu_handler = MenuHandler()
        self.pub = rospy.Publisher(topic, PoseStamped, queue_size=1)
        rospy.loginfo("Publishing posestampeds at topic: " +
                      str(self.pub.resolved_name))
        pose = Pose()
        pose.position.x = initial_x
        pose.position.y = initial_y
        pose.position.z = initial_z

        quat = quaternion_from_euler(initial_roll, initial_pitch, initial_yaw)
        pose.orientation = Quaternion(*quat)
        self.initial_pose = pose
        self.frame_id = frame_id

        # self.makeMenuMarker(pose)

        self.makeGraspIM(pose)

        self.server.applyChanges()

    def processFeedback(self, feedback):
        """
        :type feedback: InteractiveMarkerFeedback
        """
        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"

        mp = ""
        if feedback.mouse_point_valid:
            mp = " at " + str(feedback.mouse_point.x)
            mp += ", " + str(feedback.mouse_point.y)
            mp += ", " + str(feedback.mouse_point.z)
            mp += " in frame " + feedback.header.frame_id

        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo(s + ": button click" + mp + ".")
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            rospy.loginfo(s + ": menu item " +
                          str(feedback.menu_entry_id) + " clicked" + mp + ".")
            # When clicking this event triggers!
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            rospy.loginfo(s + ": pose changed")
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.pose = feedback.pose
            self.pub.publish(ps)

    # TODO
    #          << "\nposition = "
    #          << feedback.pose.position.x
    #          << ", " << feedback.pose.position.y
    #          << ", " << feedback.pose.position.z
    #          << "\norientation = "
    #          << feedback.pose.orientation.w
    #          << ", " << feedback.pose.orientation.x
    #          << ", " << feedback.pose.orientation.y
    #          << ", " << feedback.pose.orientation.z
    #          << "\nframe: " << feedback.header.frame_id
    #          << " time: " << feedback.header.stamp.sec << "sec, "
    #          << feedback.header.stamp.nsec << " nsec" )
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            rospy.loginfo(s + ": mouse down" + mp + ".")
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            rospy.loginfo(s + ": mouse up" + mp + ".")
        self.server.applyChanges()

    def makeArrow(self, msg):
        marker = Marker()

        marker.type = Marker.ARROW
        marker.scale.x = 0.15  # msg.scale * 0.3
        marker.scale.y = 0.08  # msg.scale * 0.1
        marker.scale.z = 0.03  # msg.scale * 0.03
        marker.color.r = 0.3
        marker.color.g = 0.3
        marker.color.b = 0.5
        marker.color.a = 1.0

        return marker

    def makeBoxControl(self, msg):
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self.makeArrow(msg))
        msg.controls.append(control)
        return control

    def makeGraspIM(self, pose):
        """
        :type pose: Pose
        """
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.frame_id
        int_marker.pose = pose
        int_marker.scale = 0.3

        int_marker.name = "6dof_eef"
        int_marker.description = ""  # "EEF 6DOF control"

        # insert a box, well, an arrow
        self.makeBoxControl(int_marker)
        int_marker.controls[
            0].interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_3d"
        control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        int_marker.controls.append(control)

        # self.menu_handler.insert("Do stuff", callback=self.processFeedback)

        # This makes the floating text appear
        # make one control using default visuals
        # control = InteractiveMarkerControl()
        # control.interaction_mode = InteractiveMarkerControl.MENU
        # control.description="Menu"
        # control.name = "menu_only_control"
        # int_marker.controls.append(copy.deepcopy(control))

        # marker = self.makeArrow( int_marker )
        # control.markers.append( marker )
        # control.always_visible = False
        # int_marker.controls.append(control)

        self.server.insert(int_marker, self.processFeedback)
        # self.menu_handler.apply(self.server, int_marker.name)


if __name__ == "__main__":
    rospy.init_node("pose_stamped_im")
    ig = InteractiveMarkerPoseStampedPublisher('/pr2_go_to_topic_pose/goal_pose',
                                               0.0, 0.0, 0.0,
                                               0.0, 0.0, 0.0,
                                               'base_link')

    rospy.spin()
