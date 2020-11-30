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
from logger.data_logger import data_logger
from logger.data_logger import data_reader
from logger.data_logger import log_reader_comparator
from logger.data_logger import log_hsrb_reader
import hypothesis.strategies as st

class Base:
    @pytest.fixture(autouse=True)
    def set_up(self):
        self.config = Configuration()
                
@pytest.mark.usefixtures('set_up')         
class TestPickAction(Base):
    
    @pytest.fixture()
    def randomizer(self):
        def _parameters(min_val, max_val):
            val = np.random.randint(min_val, max_val)
            return val
        return _parameters

    def test_set_up(self,randomizer):
        """Initializing the test scenario
        """  
        rospy.init_node('pick_test', anonymous=True)
        mo = Model('glass')   
        hx,hy,hz = mo.lucy_pos()[0],mo.lucy_pos()[1],mo.lucy_pos()[2]
        base_obj = Model('minicoffeetable', hx+0.7, hy, hz)
        base_obj.insert_model()
        pick_obj = Model('glass', hx+0.65, hy, 0.43)
        pick_obj.insert_model() 
        first_case = randomizer(-3, 3)

        
    def test_pick_action_activation(self):
        """Activating the pick action test and checking whether it was successful.
        """
        mo = Model('glass')   
        data_logger('logger/logs/pick_action_start')
        hx,hy,hz = mo.lucy_pos()[0],mo.lucy_pos()[1],mo.lucy_pos()[2] 
        # result = picker_client(hx+0.7, hy, 0.43, 0.0, 0.0, 0.0)
        result = MoveItPickAndPlace( pick_x = hx+0.65, pick_y = hy, pick_z = 0.5, 
                                     place_x = hx+0.7, place_y = hy, place_z = 0.76)
        data_logger('logger/logs/pick_action_end')
        
    def test_verify_object_ahead_exists(self):
        """Verification if object is ahead.
        """  
        log, lucy_log = data_reader('logger/logs/pick_action_end')
        log = log.set_index("Models")
        hsrb = log.loc["hsrb"]
        hsrb = hsrb.values.tolist()[1:] 
        log = log.drop("hsrb", axis=0)
        log = log.drop("ground_plane", axis=0)
        log = log.drop("lab", axis=0)

        x = log['X-pos'].values.tolist()
        x_res = any(hsrb[0]-0.5 <= ele <= hsrb[0]+0.5 for ele in x)

        y = log['Y-pos'].values.tolist()
        y_res = any(hsrb[1]-0.5 <= ele <= hsrb[1]+0.5 for ele in y)

        assert True == x_res and y_res  
    
    @pytest.mark.xfail(reason="One of the objects position should have changed")
    def test_allmodels_positon(self):
        """ Checking if the position of objects changed during pick action i.e. Lucy collided with an obstacle.
        """    
        lower_tolerance_difference, upper_tolerance_difference = log_reader_comparator('X-pos', 'pick_action_start', 'pick_action_end')
        assert lower_tolerance_difference == upper_tolerance_difference
        lower_tolerance_difference, upper_tolerance_difference = log_reader_comparator('Y-pos', 'pick_action_start', 'pick_action_end')
        assert lower_tolerance_difference == upper_tolerance_difference
        lower_tolerance_difference, upper_tolerance_difference = log_reader_comparator('Z-pos', 'pick_action_start', 'pick_action_end')
        assert lower_tolerance_difference == upper_tolerance_difference
        
    def test_tear_down(self):
        """Tearing down the setup for the pick test.
        """  
        test = Model('glass')   
        test.delete_model('glass')
        test.delete_model('minicoffeetable')