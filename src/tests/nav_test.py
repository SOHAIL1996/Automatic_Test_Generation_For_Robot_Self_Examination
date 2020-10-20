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
from scen_gen.random_scenario_generator import Model
from logger.data_logger import data_logger
from logger.data_logger import data_reader
from logger.data_logger import log_reader
from logger.data_logger import log_hsrb_reader
from hypothesis import given, settings, Verbosity, example
import hypothesis.strategies as st

#################################################################################################
big_models = ['table','shelf','cabinet','sofa']
#################################################################################################
   
# Checking if test module starts up
def test_startup_check():
    assert 1 == 1

# def test_scenario_generation(): 
#     # destination = random.choice(big_models)
#     destination = 'table'
#     data_logger('logger/logs/nav_start')
#     result = navi_action_client(destination)
#     data_logger('logger/logs/nav_end')
#     assert result == True
    
# (Property_test) Checking if the position of objects changed
def test_allmodels_positon_x():
                
    expected_difference, original_difference = log_reader('X-pos')
    assert expected_difference == original_difference

# (Property_test) Checking if the position of objects changed
def test_allmodels_positon_y():
    
    expected_difference, original_difference = log_reader('Y-pos')
    assert expected_difference == original_difference

# (Property_test) Checking if the position of objects changed
def test_allmodels_positon_z():
    
    expected_difference, original_difference = log_reader('Z-pos')
    assert expected_difference == original_difference

# # (Property_test) Checking if the robot is where it should be
def test_robot_position():
    log_hsrb_reader()

# # (Property_test) Check the duration of the run
# def test_navigation_duration():
#     self.nav_start_world_props.equals(self.nav_end_world_props)


#################################################################################################
# @given(st.integers(), st.integers())
# def test_sum_basic(num1, num2):
#     assert sum(num1, num2) == num1 + num2

# @given(st.integers(), st.integers())
# @example(1, 2)
# def test_sum_basic_with_example(num1, num2):
#     assert sum(num1, num2) == num1 + num2

# @settings(verbosity=Verbosity.verbose)
# @given(st.integers(), st.integers())
# def test_sum_with_property(num1, num2):
#     assert sum(num1, num2) == num1 + num2
#     # Test Identity property
#     assert sum(num1, 0) == num1
#     # Test Commutative property
#     assert sum(num1, num2) == sum(num2, num1)

# #Marking this test as expected to fail
# @pytest.mark.xfail
# @given(st.integers(), st.integers())
# def test_sum_with_shrinking_example(num1, num2):
#     assert sum(num1, num2) == num1 + num2
#     # Test Identity property
#     assert sum(num1, 0) == num1
#     # Test Commutative property
#     assert sum(num1, num2) == sum(num2, num1)
#     assert num1 <= 30
#################################################################################################
# Properties for Pick Action

# # Checking if the object is between lucys gripper
# def test_obj_pos_x(lef_gri_x, rig_grip_x, obj_x):
#     assert lef_gri_x <= obj_x <= rig_grip_x
    
# def test_obj_pos_y(lef_gri_y, rig_grip_y, obj_y):
#     assert lef_gri_y <= obj_y <= rig_grip_y
    
# # Checking if the object is not in the same position
# def test_spatial_pos_x(prev_obj_x, pres_obj_x):
#     assert prev_obj_x != pres_obj_x
    
# def test_spatial_pos_y(prev_obj_y, pres_obj_y):
#     assert prev_obj_y != pres_obj_y
    
# # Checking if the objects height is changed
# def test_obj_height(prev_obj_z, pres_obj_z):
#     assert pres_obj_z > prev_obj_z
#################################################################################################