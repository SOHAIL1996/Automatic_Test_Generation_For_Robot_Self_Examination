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
from tests.action_client.pick_client import picker_client
from tests.action_client.pick_place_client import MoveItPickAndPlace
from hypothesis import given, settings, Verbosity, example
import hypothesis.strategies as st

class Base:
    @pytest.fixture(autouse=True)
    def set_up(self):
        self.config = Configuration()
                
@pytest.mark.usefixtures('set_up')         
class TestPickAction(Base):

    def test_startup_roscore(self):
        """Initializing pick test roscore.
        """  
        rospy.init_node('pick_test', anonymous=True)
        
    def test_object_placement(self):
        """Obstacle placement for the pick test.
        """  
        mo = Model('glass')   
        hx,hy,hz = mo.lucy_pos()[0],mo.lucy_pos()[1],mo.lucy_pos()[2]
        base_obj = Model('minicoffeetable', hx+1.3, hy, hz)
        base_obj.insert_model()
        pick_obj = Model('glass', hx+1.1, hy, 0.73)
        pick_obj.insert_model() 
        
    def test_pick_action_activation(self):
        """Pick action.
        """
        mo = Model('glass')   
        hx,hy,hz = mo.lucy_pos()[0],mo.lucy_pos()[1],mo.lucy_pos()[2] 
        result = picker_client(0.45, 0.078, 0.825, 0.0, 0.0, 0.0)
        # result = MoveItPickAndPlace( pick_x = 0.518, pick_y = 0.078, pick_z = 0.825, 
        #                              place_x = hx+1.25, place_y = hy, place_z = 0.76) 
        
    def test_object_removal(self):
        """Obstacle placement for the pick test.
        """  
        test = Model('glass')   
        test.delete_model('glass')
        test.delete_model('minicoffeetable')