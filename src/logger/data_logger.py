#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
---------------------------------------------------- 
Logs the temporal properties of the models.

Starter module for the source code.
----------------------------------------------------
Supervisor: Prof. Dr. Paul Ploger
            Prof. Dr. Nico Hochgeschwender
            Alex Mitrevski 

Author    : Salman Omar Sohail
----------------------------------------------------
Date: August 19, 2020
----------------------------------------------------
"""
import numpy as np
import pandas as pd
import rospy
import std_srvs.srv as std_srvs
import gazebo_msgs.srv as gazebo_srvs
from geometry_msgs.msg import *
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.srv import GetWorldProperties

def world_state():
        """It extracts the information of all models in the gazebo world.

        Returns:
            [list]: Returns list of all relevant information
        """        
        logs = [['Models', 'X-pos','Y-pos','Z-pos','Q-1','Q-2','Q-3','Q-4']]
        lucy_logs = [['Gripper Position', 'X-pos','Y-pos','Z-pos','Q-1','Q-2','Q-3','Q-4']]
        rospy.wait_for_service("/gazebo/get_world_properties")
        rospy.wait_for_service("/gazebo/get_model_state")
        rospy.wait_for_service("/gazebo/get_link_state")
        try:
            # print(colored('Acquiring Model State','yellow'))
            
            world = rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)
            model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            end_of_effector_left = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
            end_of_effector_right= rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

            world_props = world()
            lucy_lef_gripper = end_of_effector_left('hsrb::hand_l_distal_link','world')
            lucy_rig_gripper = end_of_effector_right('hsrb::hand_r_distal_link','world')
            for objs in world_props.model_names:
                coordinates = model(objs,'world')
                temp_data   = [objs,np.around(coordinates.pose.position.x,3),
                               np.around(coordinates.pose.position.y,3),
                               np.around(coordinates.pose.position.z,3),
                               np.around(coordinates.pose.orientation.x,3),
                               np.around(coordinates.pose.orientation.y,3),
                               np.around(coordinates.pose.orientation.z,3),
                               np.around(coordinates.pose.orientation.w,3)]
                logs.append(temp_data)    
                

            lucy_data_lef  = ['Lucy left gripper',np.around(lucy_lef_gripper.link_state.pose.position.x,3),
                            np.around(lucy_lef_gripper.link_state.pose.position.y,3),
                            np.around(lucy_lef_gripper.link_state.pose.position.z,3),
                            np.around(lucy_lef_gripper.link_state.pose.orientation.x,3),
                            np.around(lucy_lef_gripper.link_state.pose.orientation.y,3),
                            np.around(lucy_lef_gripper.link_state.pose.orientation.z,3),
                            np.around(lucy_lef_gripper.link_state.pose.orientation.w,3)]
            
            lucy_data_rig  = ['Lucy right gripper',np.around(lucy_rig_gripper.link_state.pose.position.x,3),
                            np.around(lucy_rig_gripper.link_state.pose.position.y,3),
                            np.around(lucy_rig_gripper.link_state.pose.position.z,3),
                            np.around(lucy_rig_gripper.link_state.pose.orientation.x,3),
                            np.around(lucy_rig_gripper.link_state.pose.orientation.y,3),
                            np.around(lucy_rig_gripper.link_state.pose.orientation.z,3),
                            np.around(lucy_rig_gripper.link_state.pose.orientation.w,3)]
            
            lucy_logs.append(lucy_data_lef)
            lucy_logs.append(lucy_data_rig) 
            
            # print(colored('Successfully acquired Model State','green')) 
            return logs, lucy_logs
        except rospy.ServiceException as e:
            print(colored('Cannot acquire Model State.','red')) 
        return 

def data_logger(loc):
    """Writes data into a csv file.

    Args:
        loc (str): file path.
    """    
    logs, lucy_logs = world_state()
    
    df_1 = pd.DataFrame(data=logs[1:],columns=logs[0])
    df_2 = pd.DataFrame(data=lucy_logs[1:],columns=lucy_logs[0])
    
    df_1.to_csv(loc +'_logs.csv', mode= 'w' ,encoding='utf-8')    
    df_2.to_csv(loc +'_lucy_logs.csv', mode= 'w' ,encoding='utf-8')    
    
def data_reader(loc):
    """Reads .csv files and returns two data frames, one is 
    all the models and their positions and second is the lucy's
    left and right gripper positon and orientation.

    Args:
        loc (str): code name for calling the log file

    Returns:
        df_1 data_frame: Position and orientation of all the models.
        df_2 data_frame: Position and orientation of Lucys left and right gripper.
    """    
    df_1 = pd.read_csv(loc +'_logs.csv')
    df_2 = pd.read_csv(loc +'_lucy_logs.csv')
    return df_1, df_2

def log_reader_comparator(loc, action_start, action_end):
    """Reads nav files for testing purposes.

    Args:
        loc (str): the column to be compared in testing
        
        action (str): the filename of the action test e.g. nav_start, nav_end
        
    Returns:
        int: Returns 2 numbers which are the difference in the log file,
             note the robot change in position in not taken into consideration.
    """    
    nswp, ignore = data_reader('logger/logs/'+ action_start)
    newp, ignore = data_reader('logger/logs/'+ action_end)
           
    nswp = nswp.replace('-0.000', '0.000')
    newp = newp.replace('-0.000', '0.000')
    
    newp = newp.set_index("Models")
    newp = newp.drop("hsrb", axis=0)
    nswp = nswp.set_index("Models")
    nswp = nswp.drop("hsrb", axis=0)

    newp['8'] = np.where(nswp[loc]-0.5<=newp[loc], 1, 0) 
    newp['9'] = np.where(newp[loc]<=nswp[loc]+0.5, 1, 0)
             
    expected_difference_lower_tolerance = newp['8'].sum()
    original_difference_lower_tolerance = len(newp['8'])
    if expected_difference_lower_tolerance == original_difference_lower_tolerance:
        low = True
    else: 
        low = False
    
    expected_difference_upper_tolerance = newp['9'].sum()
    original_difference_upper_tolerance = len(newp['9'])
    
    if expected_difference_upper_tolerance == original_difference_upper_tolerance:
        up = True
    else: 
        up = False
    
    return low,up

def log_hsrb_reader():
    """Extracts the HSRB robots location

    Returns:
        list: returns the x y z quat1 quat2 quat3 quat4
    """    
    nav_start_world_props, ignore = data_reader('logger/logs/nav_start')
    nav_end_world_props, ignore = data_reader('logger/logs/nav_end')
    
    nav_end_world_props = nav_end_world_props.set_index("Models")
    hsrb = nav_end_world_props.loc["hsrb"]
    hsrb = hsrb.values.tolist()[1:] 
    return hsrb