#!/usr/bin/env python

import os
import csv
import numpy as np

import rospy
import roslib
import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Path

from gem_interfaces.msg import TrackPathAction, TrackPathGoal


class TrackPathClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient(
            'track_path', TrackPathAction)

        rospy.loginfo("Waiting for server")
        self.client.wait_for_server()
        rospy.loginfo("Server found")

        self.path_pub = rospy.Publisher('path', Path, queue_size=10)

        # Give some time to finish initialization subroutines
        rospy.sleep(1.0)

    def send_goal(self):
        # Create Goal msg
        dirname = os.path.dirname(__file__)
        filename = os.path.join(
            dirname, '../../../POLARIS_GEM_e2/polaris_gem_drivers_sim/gem_pure_pursuit_sim/waypoints/wps.csv')

        self.goal_msg = TrackPathGoal()
        self.goal_msg.path.header.frame_id = 'odom'

        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]

        for i in range(675, len(path_points)):
            if i % 25 == 0:
                pose = PoseStamped()
                pose.header.frame_id = 'odom'

                pose.pose.position.x = float(path_points[i][0])
                pose.pose.position.y = float(path_points[i][1])

                quat = quaternion_from_euler(0, 0, float(path_points[i][2]))
                pose.pose.orientation.x = quat[0]
                pose.pose.orientation.y = quat[1]
                pose.pose.orientation.z = quat[2]
                pose.pose.orientation.w = quat[3]

                self.goal_msg.path.poses.append(pose)
        
        # Publish path msg for debugging
        self.path_pub.publish(client.goal_msg.path)

        self.client.send_goal(self.goal_msg)


if __name__ == '__main__':
    rospy.init_node('track_path_client')
    client = TrackPathClient()
    client.send_goal()
    rospy.spin()
