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
import actionlib

from mdr_pickup_action.msg import PickupAction, PickupGoal
from tests.obstacle_generator.obstacle_gen import Model
from tests.file_reader.file_reader import Configuration
from tests.world_properties.world_prop import world_state
from tests.action_client.perceive_client import perceive_client
from hypothesis import given, settings, Verbosity, example
import hypothesis.strategies as st

c = Configuration()

def num_gen(min_val, max_val):
    return np.random.randint(min_val, max_val)

def test_startup_roscore():
    """Initializing perceive test roscore.
    """  
    rospy.init_node('perceive_test')
    
def test_Object_placement():
    """Obstacle placement for the pick test.
    """  
    mo = Model('glass')   
    hx,hy,hz = mo.lucy_pos()[0],mo.lucy_pos()[1],mo.lucy_pos()[2]
    base_obj = Model('coffeetable', hx+0.8, hy, hz)
    base_obj.insert_model()
    pick_obj = Model('glass', hx+0.6, hy, 0.72)
    pick_obj.insert_model() 
    
def test_perceive_action():
    """Perceive action.
    """
    result = perceive_client()
    assert True == result
    
def test_Object_removal():
    """Obstacle placement for the pick test.
    """  
    test = Model('glass')   
    test.delete_model('glass')
    test.delete_model('coffeetable')