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

from utilities.utility import Configuration
from scen_gen.model_placement import model_placement
from logger.data_logger import data_reader
from termcolor import colored


print(colored('Starting HSR Simulator', 'green'))
try:
    conf = Configuration()
    hsr_node = subprocess.Popen(['roslaunch', conf.rospakg, conf.ros_file_name])
    model_placement()
    # nav_node = subprocess.Popen(['roslaunch', conf.rospakg, 'nav.launch'])

finally:
    time.sleep(6000)
    hsr_node.terminate() 
    # nav_node.terminate()  
    print(colored('Terminating ros!','red'))    
