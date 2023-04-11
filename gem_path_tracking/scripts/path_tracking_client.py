#!/usr/bin/env python

import rospy
import roslib
import actionlib

from gem_interfaces.msg import TrackPathAction, TrackPathGoal


class TrackPathClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient(
            'track_path', TrackPathAction)

        rospy.loginfo("Waiting for server")
        self.client.wait_for_server()
        rospy.loginfo("Server found")

    def send_goal(self):
        self.goal_msg = TrackPathGoal()
        self.client.send_goal(self.goal_msg)
        self.client.wait_for_result(rospy.Duration.from_sec(5.0))


if __name__ == '__main__':
    rospy.init_node('track_path_client')
    server = TrackPathClient()
    server.send_goal()
