#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
---------------------------------------------------- 
Action Tests

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
import numpy as np
import pandas as pd
from subprocess import check_output
import pytest
from termcolor import colored

from utilities.utility import navi_action_client
from utilities.utility import pose_action_client
from scen_gen.random_scenario_generator import Model
from logger.data_logger import data_logger
from logger.data_logger import data_reader
from logger.data_logger import log_reader_comparator
from logger.data_logger import log_hsrb_reader
from hypothesis import given, settings, Verbosity, example
import hypothesis.strategies as st
###########################################################
# Currently, using 1 case due to lack of processing power.
###########################################################
# @settings(max_examples=1)
# @given(st.sampled_from(['table','shelf','cabinet','sofa']))
# def test_startup_check(coord_x):
#     """Checking if test module starts up
#     """  
#     print(coord_x) 
#     assert 1 == 1
###########################################################
# @settings(max_examples=1)
# @given(st.sampled_from(['table','shelf','cabinet','sofa']))
# def test_scenario_generation_map(): 
#     """Defines a scenario for the rest of the tests to run in using navigation map.
#     """    
#     # destination = random.choice(big_models)
#     destination = 'table'
#     data_logger('logger/logs/nav_start')
#     result = navi_action_client(destination)
#     data_logger('logger/logs/nav_end')
#     assert result == True
###########################################################
@settings(max_examples=1)
@given(st.floats(min_value=-4.5,max_value=4.5,allow_nan=False,allow_infinity=False),
       st.floats(min_value=-4.5,max_value=4.5,allow_nan=False,allow_infinity=False),
       st.floats(min_value=0,max_value=360,allow_nan=False,allow_infinity=False))
def test_scenario_generation_coordinates(coord_x, coord_y, direction): 
    """Defines a scenario for the rest of the tests to run in using coodrinates.
    """    
    data_logger('logger/logs/nav_start')
    result = pose_action_client(coord_x, coord_y, direction)
    data_logger('logger/logs/nav_end')
    assert result == True    
###########################################################
def test_allmodels_positon():
    """(Property_test) Checking if the position of objects changed.
    """    
    lower_tolerance_difference, upper_tolerance_difference = log_reader_comparator('X-pos')
    assert lower_tolerance_difference == upper_tolerance_difference
    lower_tolerance_difference, upper_tolerance_difference = log_reader_comparator('Y-pos')
    assert lower_tolerance_difference == upper_tolerance_difference
    lower_tolerance_difference, upper_tolerance_difference = log_reader_comparator('Z-pos')
    assert lower_tolerance_difference == upper_tolerance_difference
###########################################################
def test_robot_position():
    """(Property_test) Checking if the projected position of the robot matches 
    the position in the simulator.
    """    
    log_hsrb_reader()