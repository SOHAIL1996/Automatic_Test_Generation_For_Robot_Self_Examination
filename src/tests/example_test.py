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
        a = int(self.config.num_of_obstacles_for_nav)
        print(a)   

        # Adding the configuration file
        # data = {'First Column Name':  ['First value', 'Second value'],
        # 'Second Column Name': ['First value', 'Second value']}
        # df = pd.DataFrame (data, columns = ['First Column Name','Second Column Name'])
        # data = df.to_csv(index=False)
        # allure.attach(data, 'Configuration', allure.attachment_type.CSV)
