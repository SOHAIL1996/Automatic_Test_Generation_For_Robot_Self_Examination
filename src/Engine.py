#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
---------------------------------------------------- 
Engine

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
import os
import time
import random
import subprocess
import numpy as np

import rospy
from scen_gen.random_scenario_generator import World
from scen_gen.random_scenario_generator import Model
from utilities.utility import bound_box
from utilities.utility import Configuration
from utilities.utility import choices
from utilities.utility import collision_checker
from termcolor import colored


conf = Configuration()
world = World()

dynamic_model_tracer = []

print(colored('Starting HSR Simulator', 'green'))
try:
    hsr_node = subprocess.Popen(['roslaunch', conf.rospakg, conf.ros_file_name])
    
    # Static model placement
    static_model = Model('table',0,4,0.0254)
    static_model.insert_model()
    
    static_model_shelf = Model('shelf',4.8, 3, 0.0254, 0, 0, 1.5708)
    static_model_shelf.insert_model()
    
    static_model_sofa = Model('sofa',-3.7, -4.8, 0.0254, 0, 0, 1.5708)
    static_model_sofa.insert_model()
    
    static_model_cabinet = Model('cabinet',4.5, -4.8, 0.0254, 0, 0, 1.5708)
    static_model_cabinet.insert_model()
    
    time.sleep(1)

    # Dynamic model placement
    model_choices = choices(conf.model_list(), k=int(conf.num_of_mod))
    
    # Ensuring the models spawns in the vicinity of the static model in this case the table
    minx = np.around(static_model.bounding_box[0]*0.8+float(static_model.x_coord),2)
    maxx = np.around(static_model.bounding_box[1]*0.8+float(static_model.x_coord),2)
    miny = np.around(static_model.bounding_box[2]*0.8+float(static_model.y_coord),2)
    maxy = np.around(static_model.bounding_box[3]*0.8+float(static_model.y_coord),2)
    minz = np.around(static_model.bounding_box[4]+float(static_model.z_coord),2)
    maxz = np.around(static_model.bounding_box[5]+float(static_model.z_coord),2)

    for model in model_choices:
        dynamic_model = Model(model)
                                
        # Ensuring models don't spawn on top of existing models
        for iteration in range(0,1000):
            
            x = random.uniform(minx, maxx)
            y = random.uniform(miny, maxy)
            z = maxz + 0.03

            check = collision_checker(dynamic_model,x,y,z,dynamic_model_tracer)
            if check == False:
                
                dynamic_model.x_coord = x
                dynamic_model.y_coord = y
                dynamic_model.z_coord = z
                
                dynamic_model.insert_model()
                    
                dynamic_model_tracer.append(dynamic_model)
                break
            
            if iteration == 999:
                print(colored('No space for model insertion ','red'),dynamic_model.model_real_name)
                continue
            
    print(colored('Completed spawning dynamic models','cyan'))
    
    time.sleep(2)
    # nav_node = subprocess.Popen(['roslaunch', conf.rospakg, 'nav.launch'])

finally:
    time.sleep(6000)
    hsr_node.terminate() 
    # nav_node.terminate()  
    print(colored('Terminating ros!','red'))    
