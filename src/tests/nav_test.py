#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
---------------------------------------------------- 
Navigation Tests

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

from tests.action_client.nav_client import navi_action_client
from tests.action_client.nav_client import pose_action_client
from utilities.Omni_base_locator.oml import OmniListener
from tests.obstacle_generator.obstacle_gen import Model
from pdf_gen.pdf_creator import PdfGenerator
# from tests.file_reader.file_reader import 
from logger.data_logger import data_logger
from logger.data_logger import data_reader
from logger.data_logger import log_reader_comparator
from logger.data_logger import log_hsrb_reader
from hypothesis import given, settings, Verbosity, example
import hypothesis.strategies as st

global tracker
global start_time
start_time = 0.0

def test_startup_check():
    """Initializing nav test roscore.
    """  
    start_time = time.time()
    rospy.init_node('nav_test')

@settings(max_examples=1)
@given(st.sampled_from(['minicoffeetable']))  
def test_obstacle_placement(obstacle):
    """Obstacle placement for the navigation test.
    """  
    number_of_obstacles = np.random.randint(1,15)
    store = [[0,0]]
    for i in range(number_of_obstacles):
        x,y,z = np.random.randint(-3,3),np.random.randint(-3,3),0
        if x == 0 and y==0:
            continue
        store.append([x,y])
        for i in store:
            if [x,y] == i:
                continue
        obstacles = Model(obstacle,x,y,z)
        obstacles.insert_model()
        
# @settings(max_examples=1)
# @given(st.sampled_from(['table','shelf','cabinet','sofa']))
# def test_scenario_generation_map(destination): 
#     """Defines a scenario for the rest of the tests to run in using navigation map.
#     """    
#     data_logger('logger/logs/nav_start')
#     result = navi_action_client(destination)
#     data_logger('logger/logs/nav_end')
#     assert result == True

@settings(max_examples=1)
@given(st.floats(min_value=-4.5,max_value=4.5,allow_nan=False,allow_infinity=False).filter(lambda x:x>1),
       st.floats(min_value=-4.5,max_value=4.5,allow_nan=False,allow_infinity=False),
       st.integers(min_value=1,max_value=360))
def test_scenario_generation_coordinates(coord_x, coord_y, direction): 
    """Defines a scenario for the rest of the tests to run in using coodrinates.
    """    
    coord_x, coord_y, direction = np.random.randint(-2,2),np.random.randint(-2,2),np.random.randint(0,360)
    data_logger('logger/logs/nav_start')
    result = pose_action_client(coord_x, coord_y, direction)
    data_logger('logger/logs/nav_end')
    assert result == True    

def test_allmodels_positon():
    """(Property_test) Checking if the position of objects changed.
    """    
    lower_tolerance_difference, upper_tolerance_difference = log_reader_comparator('X-pos')
    assert lower_tolerance_difference == upper_tolerance_difference
    lower_tolerance_difference, upper_tolerance_difference = log_reader_comparator('Y-pos')
    assert lower_tolerance_difference == upper_tolerance_difference
    lower_tolerance_difference, upper_tolerance_difference = log_reader_comparator('Z-pos')
    assert lower_tolerance_difference == upper_tolerance_difference

def test_robot_position():
    """(Property_test) Checking if the projected position of the robot matches 
    the position in the simulator.
    """    
    hx,hy,hz = log_hsrb_reader()[0], log_hsrb_reader()[1], log_hsrb_reader()[2]
    omni = OmniListener()
    omni.omnibase_listener()
    x,y,z = omni.x, omni.y, omni.z

    assert hx-0.5 <= x <= hx+0.5
    assert hy-0.5 <= y <= hy+0.5

def test_legal_zone():
    """(Property_test) Checking if the projected position of the robot has not 
    gone out of the boundary. The boundary is the test lab navigation map and its 
    dimensions are 10x10 m^2.
    """    
    hx,hy,hz = log_hsrb_reader()[0], log_hsrb_reader()[1], log_hsrb_reader()[2]
    omni = OmniListener()
    omni.omnibase_listener()
    x,y,z = omni.x, omni.y, omni.z

    assert -5 <= hx <= 5
    assert -5 <= hy <= 5
    assert -5 <= x <= 5
    assert -5 <= y <= 5

def test_evaluation_sheet():
    pdf = PdfGenerator('Navigation Test')
    
    # pdf.robot_nav_time = time.time() - start_time
    
    pdf.pdf_creation()
    