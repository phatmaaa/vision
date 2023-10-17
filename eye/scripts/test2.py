#!/usr/bin/env python

import rospy
import actionlib
from eye.msg import VisionAction, VisionGoal, VisionResult
from constants import *
import json


class Node():
    def request_action(self,action):
        client = actionlib.SimpleActionClient('vision',VisionAction)
        # rospy.sleep(2)
        client.wait_for_server()

        msg = VisionGoal()
        req_dict = {'header': action.name}

        if action == Action.captureImage:
            req_dict['data']='capture-100'
        if action == Action.detectHole: 
            req_dict['data']='Hole-100'
        if action == Action.detectFiducial:
            req_dict['data']='Fiducial-100'

        msg.request = json.dumps(req_dict)

        client.send_goal(msg)

        if client.wait_for_result(rospy.Duration.from_sec(5.0)):
            result_dict = json.loads(client.get_result().result)
            return result_dict['success'],result_dict['feedback']

        return False, 0

    def __init__(self):
        # rospy.init_node('test_node_2')
        self.rate = rospy.Rate(50)
        self.feedback = ''

        while not rospy.is_shutdown():
            self.rate.sleep()
            # print (self.request_action(Action.captureImage))
            # rospy.sleep(3.0)
            # print("******img*******")
            result, feedback = self.request_action(Action.detectFiducial)
            print(result)
            if (result == True):
                rospy.loginfo(feedback)
                self.feedback = feedback
                rospy.sleep(2.0)
                print("=====ArUco======")
                break
            result, feedback = self.request_action(Action.detectHole)#added if statment (hole detection) 
            print(result)
            if (result == True):
                rospy.loginfo(feedback)
                self.feedback = feedback
                rospy.sleep(2.0)
                print("=====ArUco======")
                break
            elif(result == False or not(self.feedback)):
                continue
                
            # print(self.request_action(Action.detectHole))
            # rospy.sleep(2.0)
            # print("oooooHoleooooooo")


if __name__ == "__main__":
    Node()
