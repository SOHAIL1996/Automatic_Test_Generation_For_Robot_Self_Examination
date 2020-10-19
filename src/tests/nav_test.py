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
from subprocess import check_output
import pytest
from termcolor import colored

from utilities.utility import navi_action_client
from scen_gen.random_scenario_generator import Model
from logger.data_logger import data_logger
from logger.data_logger import data_reader
from hypothesis import given, settings, Verbosity, example
import hypothesis.strategies as st

#################################################################################################
# Strategies
# 'nothing','just', 'one_of','none','choices', 'streaming','booleans', 'integers',
# 'floats', 'complex_numbers', 'fractions','decimals','characters', 'text', 'from_regex',
# 'binary', 'uuids','tuples', 'lists', 'sets', 'frozensets', 'iterables','dictionaries',
# 'fixed_dictionaries','sampled_from', 'permutations','datetimes', 'dates', 'times', 
# 'timedeltas','builds','randoms', 'random_module','recursive', 'composite','shared', 
# 'runner', 'data','deferred','from_type', 'register_type_strategy', 'emails'
#################################################################################################               
# Input type
# check pre conditions
# Ensure property is satisfied
# Generalized property
#################################################################################################
class NavTest():
    """
    This class contains all relevant information of a gazebo model.
    """    
    def __init__(self, destination):
        
        data_logger('nav_start')
        self.nav_start_world_props, ignore = data_reader('nav_start')
        
        result = navi_action_client('table')
        print(colored('Navigation completed.','blue')) 
        
        data_logger('nav_end')
        self.nav_end_world_props, ignore = data_reader('nav_end')
        print(self.nav_end_world_props)
      
    # Checking if the position of objects changed
    # def test_obj_pos_x(lef_gri_x, rig_grip_x, obj_x):
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