#!/usr/bin/env python

import random
import math

import rospy
from gazebo_msgs.msg import ModelStates, ModelState


class MoveObstacles:
    def __init__(self):
        self.gz_model_states_sub = rospy.Subscriber(
            '/gazebo/model_states', ModelStates, self.gz_model_states_cb)

        self.gz_model_state_pub = rospy.Publisher(
            '/gazebo/set_model_state', ModelState, queue_size=10)

        self.gz_models = {}
        self.models_to_be_updated = set()

    def gz_model_states_cb(self, msg):
        if len(self.gz_models) == 0:
            for i, name in enumerate(msg.name):
                if name.find('unit_cylinder') != -1:
                    self.gz_models.update({name: [msg.pose[i].position.x,
                                                  msg.pose[i].position.y]})
        # else:
        #     for i, name in enumerate(msg.name):
        #         if name.find('unit_cylinder') != -1:
        #             self.gz_models[name] = [msg.pose[i].position.x,
        #                                     msg.pose[i].position.y]


if __name__ == '__main__':
    rospy.init_node('move_obstacles')
    node = MoveObstacles()
    try:
        rate = rospy.Rate(20)
        i = 0
        while not rospy.is_shutdown():
            for model in node.gz_models.keys():
                model_state = ModelState()

                model_state.model_name = model
                # model_state.twist.linear.x = random.random()/4 - 1*random.randint(0, 1)
                # model_state.twist.linear.y = random.random()/4 - 1*random.randint(0, 1)
                model_state.pose.position.x = 0.4*math.cos(i) + \
                    node.gz_models[model][0]
                model_state.pose.position.y = 0.6*math.sin(i) + \
                    node.gz_models[model][1]
                model_state.pose.position.z = 0.6

                node.gz_model_state_pub.publish(model_state)
                
                i = i+1/20

            rate.sleep()
    except KeyboardInterrupt:
        pass
