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
import allure

from mdr_pickup_action.msg import PickupAction, PickupGoal
from tests.obstacle_generator.obstacle_gen import Model
from tests.file_reader.file_reader import Configuration
from tests.world_properties.world_prop import world_state
from tests.action_client.perceive_client import perceive_client
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
class TestPerception(Base):
    
    @pytest.fixture()
    def randomizer(self):
        def _parameters(min_val, max_val):
            val = np.random.randint(min_val, max_val)
            return val
        return _parameters
    
    # @allure.severity('minor')   
    def test_set_up(self,randomizer):
        """Initialzing parameters for testing.
        """        
        self.config = Configuration() 
        rospy.init_node('perception_node') 
        # Obtaining Lucy's position
        lucy_loc = Model('glass')   
        hx,hy,hz = lucy_loc.lucy_pos()[0],lucy_loc.lucy_pos()[1],lucy_loc.lucy_pos()[2]
        # Loading in a objects for perception
        base_obj = Model(self.config.Platform_for_obstacle_perc, hx+0.8, hy, hz)
        base_obj.insert_model()
        perceive_obj = Model(self.config.Obstacles_for_perc, hx+0.6, hy, 0.72)
        perceive_obj.insert_model() 
  
        # Adding the configuration file
        # data = {'First Column Name':  ['First value', 'Second value'],
        # 'Second Column Name': ['First value', 'Second value']}
        # df = pd.DataFrame (data, columns = ['First Column Name','Second Column Name'])
        # data = df.to_csv(index=False)
        # data.to_csv('my_csv.csv', mode='a', header=False)

    def test_perceive_action(self):
        """Activates perception action.
        """
        data_logger('logger/logs/perception')
        result = perceive_client()    
        assert True == result
    
    def test_verify_object_ahead_exists(self):
        """Verification if object is ahead.
        """  
        log, lucy_log = data_reader('logger/logs/perception')
        log = log.set_index("Models")
        hsrb = log.loc["hsrb"]
        hsrb = hsrb.values.tolist()[1:] 
        log = log.drop("hsrb", axis=0)
        log = log.drop("ground_plane", axis=0)
        log = log.drop(self.config.World, axis=0)

        x = log['X-pos'].values.tolist()
        x_res = any(hsrb[0]-0.5 <= ele <= hsrb[0]+0.5 for ele in x)

        y = log['Y-pos'].values.tolist()
        y_res = any(hsrb[1]-0.5 <= ele <= hsrb[1]+0.5 for ele in y)

        assert True == x_res and y_res              
        
    def test_tear_down(self):
        """Obstacle removal.
        """  
        # Deleting spawned models
        test = Model('glass')  
        test.delete_model(self.config.Obstacles_for_perc)
        test.delete_model(self.config.Platform_for_obstacle_perc)
        # allure.attach(data, 'Configuration', allure.attachment_type.CSV)