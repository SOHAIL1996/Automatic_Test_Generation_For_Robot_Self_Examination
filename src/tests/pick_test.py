#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
---------------------------------------------------- 
Picking Tests

The ...
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
import rospy
import numpy as np
import pandas as pd
from subprocess import check_output
import pytest
from termcolor import colored

from utilities.utility import navi_action_client
from utilities.utility import pose_action_client
from utilities.Omni_base_locator.oml import OmniListener
from tests.obstacle_generator.obstacle_gen import Model
# from tests.file_reader.file_reader import 
from logger.data_logger import data_logger
from logger.data_logger import data_reader
from logger.data_logger import log_reader_comparator
from logger.data_logger import log_hsrb_reader
from hypothesis import given, settings, Verbosity, example
import hypothesis.strategies as st

###########################################################
# Currently, using 1 case due to lack of processing power.
###########################################################

def test_startup_check():
    """Initializing pick test roscore.
    """  
    rospy.init_node('pick_test')
    
@settings(max_examples=1)
@given(st.sampled_from(['coffeetable']))  
def test_Object_placement(obstacle):
    """Obstacle placement for the navigation test.
    """  
    number_of_obstacles = np.random.randint(1,6)
    for i in range(number_of_obstacles):
        x,y,z = np.random.randint(-3,3),np.random.randint(-3,3),0
        if x == 0 or y==0:
            continue
        obstacles = Model(obstacle,x,y,z)
        obstacles.insert_model()
