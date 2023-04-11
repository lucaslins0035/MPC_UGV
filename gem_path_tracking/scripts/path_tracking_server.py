#!/usr/bin/env python

import rospy
import roslib
import actionlib

from gem_interfaces.msg import TrackPathAction


class GemMPC:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            'track_path', TrackPathAction, self.execute_action, False)
        self.server.start()
        rospy.loginfo("Server initialized")

    def execute_action(self, goal):
        rospy.loginfo("Goal received")
        
        rospy.loginfo("Path len: "+str(len(goal.path.poses)))
        
        self.server.set_succeeded()
        rospy.loginfo("Goal finished")
        


if __name__ == '__main__':
    rospy.init_node('track_path_server')
    server = GemMPC()
    rospy.spin()
