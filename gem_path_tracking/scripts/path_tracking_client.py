#!/usr/bin/env python

import os
import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

import rospy
import roslib
import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Path

from gem_interfaces.msg import TrackPathAction, TrackPathGoal

mpl.rcParams['font.size'] = 18
mpl.rcParams['lines.linewidth'] = 3
mpl.rcParams['axes.grid'] = True


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

        for i in range(675, 850):
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

    client.client.wait_for_result()

    result_msg = client.client.get_result()
    
    # Plot path tracking error
    fig = plt.figure(figsize=(12, 12))
    ax = fig.add_subplot(1, 1, 1)

    major_ticks = np.arange(0, 1.25, 0.25)
    minor_ticks = np.arange(0, 1.25, 0.05)
    ax.set_yticks(major_ticks)
    ax.set_yticks(minor_ticks, minor=True)

    ax.grid(which='minor', alpha=0.2)
    ax.grid(which='major', alpha=0.5)

    ax.set_title("Path Tracking Error x Time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Track Error (m)")

    ax.plot(np.arange(len(result_msg.path_tracking_error))*0.1, result_msg.path_tracking_error)
    plt.savefig('/home/lucas/polaris_ws/src/MPC_UGV/study/foo.png')
