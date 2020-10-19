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
import pandas as pd
from scen_gen.random_scenario_generator import Model
from scen_gen.random_scenario_generator import World

def data_logger(time):
    world = World()
    logs, lucy_logs = world.world_state()
    
    df_1 = pd.DataFrame(logs)
    df_2 = pd.DataFrame(lucy_logs)
    
    df_1.to_csv('src/logger/logs/'+time+'_logs.csv', mode= 'w' ,encoding='utf-8')    
    df_2.to_csv('src/logger/logs/'+time+'_lucy_logs.csv', mode= 'w' ,encoding='utf-8')    
    
def data_reader(time):
    """Reads .csv files and returns two data frames, one is 
    all the models and their positions and second is the lucy's
    left and right gripper positon and orientation.

    Args:
        time str: code name for calling the log file

    Returns:
        df_1 data_frame: Position and orientation of all the models.
        df_2 data_frame: Position and orientation of Lucys left and right gripper.
    """    
    df_1 = pd.read_csv('src/logger/logs/'+time+'_logs.csv')
    df_2 = pd.read_csv('src/logger/logs/'+time+'_lucy_logs.csv')
    return df_1, df_2