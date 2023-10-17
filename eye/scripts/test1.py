#!/usr/bin/env python

import rospy
import actionlib
from eye.msg import VisionAction, VisionGoal, VisionResult
from constants import *
import json


class Node():
    def callback_actionRequest(self,msg):
        
        goal_dict = json.loads(msg.request)

        data = goal_dict['data']

        if goal_dict['header'] == Action.detectHole.name:
            print("Detect hole requested ...")

            feedback_dict = {'success':True, 'feedback':"I have taken Photo"}

            msg = VisionResult()
            msg.result = json.dumps(feedback_dict)

            self.actionSrv.set_succeeded(msg)

    def __init__(self):
        rospy.init_node('test_node_1')
        self.rate = rospy.Rate(50)
        
        self.actionSrv = actionlib.SimpleActionServer('test_node_1', VisionAction, self.callback_actionRequest,False)

        self.actionSrv.start()
        

        while not rospy.is_shutdown():
            self.rate.sleep()
            pass


if __name__ == "__main__":
    Node()
