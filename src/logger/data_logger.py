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
from scen_gen.random_scenario_generator import Model
from scen_gen.random_scenario_generator import World

def data_logger(loc):
    """Writes data into a csv file.

    Args:
        loc ([str]): file path.
    """    
    world = World()
    logs, lucy_logs = world.world_state()
    
    df_1 = pd.DataFrame(data=logs[1:],columns=logs[0])
    df_2 = pd.DataFrame(data=lucy_logs[1:],columns=lucy_logs[0])
    
    df_1.to_csv(loc +'_logs.csv', mode= 'w' ,encoding='utf-8')    
    df_2.to_csv(loc +'_lucy_logs.csv', mode= 'w' ,encoding='utf-8')    
    
def data_reader(loc):
    """Reads .csv files and returns two data frames, one is 
    all the models and their positions and second is the lucy's
    left and right gripper positon and orientation.

    Args:
        loc ([str]): code name for calling the log file

    Returns:
        df_1 data_frame: Position and orientation of all the models.
        df_2 data_frame: Position and orientation of Lucys left and right gripper.
    """    
    df_1 = pd.read_csv(loc +'_logs.csv')
    df_2 = pd.read_csv(loc +'_lucy_logs.csv')
    return df_1, df_2

def log_reader(loc):
    """Reads nav files for testing purposes.

    Args:
        loc ([str]): the column to be compared in testing

    Returns:
        [int]: Returns 2 numbers which are the difference in the log file,
               note the robot change in position in not taken into consideration.
    """    
    nav_start_world_props, ignore = data_reader('logger/logs/nav_start')
    nav_end_world_props, ignore = data_reader('logger/logs/nav_end')
           
    nav_start_world_props = nav_start_world_props.replace('-0.0', '0.0')
    nav_end_world_props   = nav_end_world_props.replace('-0.0', '0.0')
    
    nav_end_world_props = nav_end_world_props.set_index("Models")
    nav_end_world_props = nav_end_world_props.drop("hsrb", axis=0)
    nav_start_world_props = nav_start_world_props.set_index("Models")
    nav_start_world_props = nav_start_world_props.drop("hsrb", axis=0)
    
    nav_end_world_props['8'] = np.where(nav_start_world_props[loc] == nav_end_world_props[loc], 1, 0)
             
    expected_difference = nav_end_world_props['8'].sum()
    original_difference = len(nav_end_world_props['8'])
    
    return expected_difference,original_difference

def log_hsrb_reader():
    """Extracts the HSRB robots location

    Returns:
        [list]: returns the x y z quat1 quat2 quat3 quat4
    """    
    nav_start_world_props, ignore = data_reader('logger/logs/nav_start')
    nav_end_world_props, ignore = data_reader('logger/logs/nav_end')
    
    nav_end_world_props = nav_end_world_props.set_index("Models")
    hsrb = nav_end_world_props.loc["hsrb"]
    hsrb = hsrb.values.tolist()[1:] 
    return hsrb