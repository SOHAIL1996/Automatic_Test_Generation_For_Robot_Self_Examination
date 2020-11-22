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
from pdf_gen.pdf_creator import PdfGenerator
from hypothesis import given, settings, Verbosity, example
import hypothesis.strategies as st

class TestPerception():
    
    def test_initialization(self):
        """Initialzing parameters for testing.
        """        
        self.pdf = PdfGenerator('Perception Test')
        self.config = Configuration()    
        
        rospy.init_node('perceive_test')
        pytest.skip('implement later')

        
    def test_Object_placement(self):
        """Obstacle placement for the pick test.
        """  
        lucy_loc = Model('glass')   
        hx,hy,hz = lucy_loc.lucy_pos()[0],lucy_loc.lucy_pos()[1],lucy_loc.lucy_pos()[2]
        base_obj = Model('coffeetable', hx+0.8, hy, hz)
        base_obj.insert_model()
        pick_obj = Model('mug', hx+0.6, hy, 0.72)
        pick_obj.insert_model() 
        
    # def test_perceive_action(self):
    #     """Perceive action.
    #     """
    #     result = perceive_client()
        
    #     self.pdf.query_1 = 'Perception action'
    #     if result == True:
    #         self.pdf.result_1 = '\u2713'
    #     else:
    #         self.pdf.result_1 = '\u2717'
    
    #     assert True == result
        
    def test_Object_removal(self):
        """Obstacle removal.
        """  
        test = Model('glass')   
        test.delete_model('mug')
        test.delete_model('coffeetable')
        
