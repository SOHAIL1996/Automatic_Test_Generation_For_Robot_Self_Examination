#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
---------------------------------------------------- 
Utilities

Utilitie functions.
----------------------------------------------------
Supervisor: Prof. Dr. Paul Ploger
            Prof. Dr. Nico Hochgeschwender
            Alex Mitrevski 

Author    : Salman Omar Sohail
----------------------------------------------------
Date: August 19, 2020
----------------------------------------------------
"""

import random
import sys,os
import numpy as np

class Configuration():
    """It extracts information from the configuration file.
    """    
    def __init__(self):
        
        self.rospakg       = 'atg'
        self.ros_file_name = 'hsr.launch'

        with open("src/utilities/config.txt",mode='r') as file:
            data    = file.readlines()
            # Index starts from 0 select -1 line
            launch_dir                  = data[4]
            model_dir                   = data[5]
            Multi_obstacle_platform     = data[7]
            num_of_mod                  = data[8]
            models_1                    = data[9]
            models_2                    = data[10]
            num_of_obstacles_for_nav    = data[11]
            Obstacles_for_nav           = data[12]
            Platform_for_obstacle_pick  = data[13]
            Obstacles_for_pick          = data[14]
            Platform_for_obstacle_perc  = data[15]
            Obstacles_for_perc          = data[16]
            World                       = data[19]

        # Splitting the text
        launch_dir                 = launch_dir.split('=')
        model_dir                  = model_dir.split('=')
        Multi_obstacle_platform    = Multi_obstacle_platform.split('=')
        num_of_mod                 = num_of_mod.split('=')
        models_1                   = models_1.split('=')
        models_2                   = models_2.split('=')
        num_of_obstacles_for_nav   = num_of_obstacles_for_nav.split('=')
        Obstacles_for_nav          = Obstacles_for_nav.split('=')
        Platform_for_obstacle_pick = Platform_for_obstacle_pick.split('=')
        Obstacles_for_pick         = Obstacles_for_pick.split('=')
        Platform_for_obstacle_perc = Platform_for_obstacle_perc.split('=')
        Obstacles_for_perc         = Obstacles_for_perc.split('=')
        World                      = World.split('=')
        # Removing the text
        self.launch_dir                 = launch_dir[1].strip()
        self.model_dir                  = model_dir[1].strip()
        self.num_of_mod                 = num_of_mod[1].strip()
        self.Multi_obstacle_platform    = Multi_obstacle_platform[1].strip()
        self.models                     = models_1[1].strip()+models_2[1].strip()
        self.num_of_obstacles_for_nav   = num_of_obstacles_for_nav[1].strip()
        self.Obstacles_for_nav          = Obstacles_for_nav[1].strip()
        self.Platform_for_obstacle_pick = Platform_for_obstacle_pick[1].strip()
        self.Obstacles_for_pick         = Obstacles_for_pick[1].strip()
        self.Platform_for_obstacle_perc = Platform_for_obstacle_perc[1].strip()
        self.Obstacles_for_perc         = Obstacles_for_perc[1].strip()
        self.World                      = World[1].strip()
            
    def config_data_frame(self):
        """[summary]
        """    

        data = {'Configuration Settings':  ['First value', 'Second value'],
                'Selected Parameters'   : ['First value', 'Second value']}

        df = pd.DataFrame (data, columns = ['Configuration Settings','Selected Parameters'])
        df.to_csv('tests/results/configuration_data.csv',index=False)