#!/usr/bin/env python
import rospy
import actionlib

from mdr_pickup_action.msg import PickupAction, PickupGoal

def picker_client(x,y,z,q1=0,q2=0,q3=0,q4=0):

    client = actionlib.SimpleActionClient('pickup_server', PickupAction)
    client.wait_for_server()

    goal = PickupGoal()
    goal.pose.header.frame_id = 'base_link'
    goal.pose.header.stamp = rospy.Time.now()

    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.position.z = z

    goal.pose.pose.orientation.x = q1
    goal.pose.pose.orientation.y = q2
    goal.pose.pose.orientation.z = q3
    goal.pose.pose.orientation.w = q4

    client.send_goal(goal)
    client.wait_for_result()

    rospy.loginfo(client.get_result())
    return True
