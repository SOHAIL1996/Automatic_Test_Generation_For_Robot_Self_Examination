#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
---------------------------------------------------- 
Toyota HSR base ROS subscriber

Subscribes to the Toyota HSR's omni base messages.
----------------------------------------------------
Supervisor: Prof. Dr. Paul Ploger
            Prof. Dr. Nico Hochgeschwender
            Alex Mitrevski 

Author    : Salman Omar Sohail
----------------------------------------------------
Date: August 19, 2020
----------------------------------------------------
"""

import numpy as np
import rospy
from control_msgs.msg import JointTrajectoryControllerState

class OmniListener():
    """Subscribes and extracts position of the omni base controller of Lucy.
    """    
    def __init__(self):
        self.x = None
        self.y = None
        self.z = None
        
    def omnibase_listener(self):
        # rospy.init_node('omnibase_listener')
        rospy.Subscriber('/hsrb/omni_base_controller/state',JointTrajectoryControllerState,self.omnibase_callback_parser)
        rospy.sleep(0.05)
         
    def omnibase_callback_parser(self,data):
        self.x = np.around(data.actual.positions[0],3)
        self.y = np.around(data.actual.positions[1],3)
        self.z = np.around(data.actual.positions[2],3)
    


