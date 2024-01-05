#!/usr/bin/env python

import os
import rospy
import sys
import random

from follow_wall.msg import State
from std_msgs.msg import String

sys.path.append(os.path.dirname(__file__))

from qtable_sarsa import q_table

class RobotState:
    def __init__(self, left=None, front=None, rightfront = None, right = None, orientation = None):
        self.left = left
        self.front = front
        self.rightfront = rightfront
        self.right = right
        self.orientation = orientation

scale = 0.45

state = RobotState()

def Q_lookup_action(prior_state):
    if prior_state.left != None and prior_state.right != None  and prior_state.rightfront != None and prior_state.right != None and prior_state.orientation != None:
        state_index =  "left-" + str(prior_state.left) + "-front-" + str(prior_state.front) + "-rightfront-" + str(prior_state.rightfront) + "-right-" + str(prior_state.right) + "-orientation-" + str(prior_state.orientation)
        if q_table[state_index]["left"] != q_table[state_index]["forward"] != q_table[state_index]["right"]:
            return max(q_table[state_index], key=q_table[state_index].get) # return action with highest reward
        else:
            return random.choice(list(q_table[state_index].keys()))
    return "State does not exist"

def actionLookupCallback(data):  # Get state from here
    if data.left_distance <= scale*0.5:
        state.left = "close"
    else:
        state.left = "far"

    if data.front_distance < scale*0.5:
        state.front = "tooclose"
    elif scale*0.5 <= data.front_distance < scale*0.6:
        state.front = "close"
    elif scale*0.6 <= data.front_distance < scale*1.2:
        state.front = "medium"
    else:
        state.front = "far"

    if data.right_front_distance <= scale*1.2:
        state.rightfront = "close" 
    else:
        state.rightfront = "far"

    if data.right_distance < scale*0.5:
        state.right = "tooclose"
    elif scale*0.5 <= data.right_distance < scale*0.6:
        state.right = "close"
    elif scale*0.6 <= data.right_distance <= scale*0.8:
        state.right = "medium"
    elif scale*0.8 <= data.right_distance <= scale*1.2:
        state.right = "far"
    else:
        state.right = "toofar"

    if  275 < data.orientation or data.orientation <= 85:
        state.orientation = "approaching"
    elif 95 < data.orientation < 265:
        state.orientation = "leaving"
    elif 85 <= data.orientation <=95 or 265 <= data.orientation <= 275:
        state.orientation = "parallel"
    else:
        state.orientation = "undefined"

def main():
    rospy.init_node("stateaction_map_node", anonymous=True)
    rate = rospy.Rate(20)  # 20hz
    rospy.Subscriber("robot_state", State, actionLookupCallback)
    pub_action = rospy.Publisher("action", String, queue_size=10)
    for i in range(100): # Need a short delay
        rate.sleep()
    while not rospy.is_shutdown():
        action = Q_lookup_action(state)
        rospy.loginfo("Optimal Action: %s", action)
        pub_action.publish(action)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
