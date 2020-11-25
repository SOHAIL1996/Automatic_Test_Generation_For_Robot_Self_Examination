#! /usr/bin/env python
import rospy
import actionlib
from mdr_perceive_plane_action.msg import PerceivePlaneAction, PerceivePlaneGoal  
from hsrb_interface import Robot
    
def perceive_client():

    robot = Robot()
    whole_body = robot.get('whole_body')
    whole_body.move_to_joint_positions({'head_tilt_joint': -0.3})
    
    client = actionlib.SimpleActionClient('/mdr_actions/perceive_plane_server', PerceivePlaneAction)
    client.wait_for_server()
    goal = PerceivePlaneGoal()
    goal.plane_config = 'table'
    goal.plane_frame_prefix = 'frame_table'
    try:
        timeout = 45.0
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
    except:
        print('No result')
    
    return True