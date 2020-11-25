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

        with open("utilities/config.txt",mode='r') as file:
            data    = file.readlines()
            # Index starts from 0 select -1 line
            launch_dir = data[4]
            model_dir  = data[5]
            num_of_mod = data[8]
            models_1   = data[9]
            models_2   = data[10]
            obstacles   = data[11]

            launch_dir             = launch_dir.split('=')
            model_dir              = model_dir.split('=')
            num_of_mod             = num_of_mod.split('=')
            models_1               = models_1.split('=')
            models_2               = models_2.split('=')
            obstacles              = obstacles.split('=')

            self.launch_dir         = launch_dir[1].strip()
            self.model_dir          = model_dir[1].strip()
            self.num_of_mod         = num_of_mod[1].strip()
            self.models             = models_1[1].strip()+models_2[1].strip()
            self.obstacles          = obstacles[1].strip()
            self.obstacles          = self.obstacles .split(',')
            
        def config_data_frame(self):
            """[summary]
            """    
                    
            data = {'Configuration Settings':  ['First value', 'Second value'],
                    'Selected Parameters'   : ['First value', 'Second value']}
            
            
            df = pd.DataFrame (data, columns = ['Configuration Settings','Selected Parameters'])
            df.to_csv('tests/results/configuration_data.csv',index=False)
            

