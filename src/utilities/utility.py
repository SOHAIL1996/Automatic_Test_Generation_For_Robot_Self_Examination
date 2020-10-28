#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
---------------------------------------------------- 
Utilities

Utilitie functions.
----------------------------------------------------
Supervisor: Prof. Dr. Paul Ploger
            Prof. Dr. Nico Hochgeschwender
            Alex Mitrevski 

Author    : Salman Omar Sohail
----------------------------------------------------
Date: August 19, 2020
----------------------------------------------------
"""

import random
import sys,os
import numpy as np
from stl import mesh
from itertools import accumulate as _accumulate, repeat as _repeat
from bisect import bisect as _bisect

import sys
import rospy
import actionlib
import std_msgs.msg

from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from mdr_move_base_action.msg import MoveBaseAction, MoveBaseGoal

class Configuration():
    """It extracts information from the configuration file.
    """    
    def __init__(self):
        
        self.rospakg       = 'atg'
        self.ros_file_name = 'hsr.launch'

        with open("src/utilities/config.txt",mode='r') as file:
            data    = file.readlines()
            # Index starts from 0 select -1 line
            launch_dir = data[4]
            model_dir  = data[5]
            num_of_mod = data[8]
            models_1   = data[9]
            models_2   = data[10]

            launch_dir             = launch_dir.split('=')
            model_dir              = model_dir.split('=')
            num_of_mod             = num_of_mod.split('=')
            models_1               = models_1.split('=')
            models_2               = models_2.split('=')

            self.launch_dir         = launch_dir[1].strip()
            self.model_dir          = model_dir[1].strip()
            self.num_of_mod         = num_of_mod[1].strip()
            self.models             = models_1[1].strip()+models_2[1].strip()

    def model_uri_correcter(self, model_name):
        """
        This function opens the sdf model files and corrects the uri tags. This is useful when changing the
        paths of the model folder. 

        Args:
            model_name (str): The model name for the uri tag correction.
        """        
        with open(os.path.realpath('.')+'/Environment/models/'+model_name+'/'+model_name+'.sdf',mode='r') as file:
                counter    = 0
                data       = file.readlines()
                save_index = []

                for word in data:
                    if (word.find('uri') != -1): 
                        save_index.append(counter) 
                    counter +=1
                
                data[save_index[0]] = '             <uri>'+self.model_dir+'/'+model_name+'/meshes/'+model_name+'.stl</uri>\n'
                data[save_index[1]] = '             <uri>'+self.model_dir+'/'+model_name+'/meshes/'+model_name+'.stl</uri>\n'
                data[save_index[2]] = '             <uri>'+self.model_dir+'/'+model_name+'/materials/scripts</uri>\n'
                data[save_index[3]] = '             <uri>'+self.model_dir+'/'+model_name+'/materials/textures</uri>\n'
        
        with open(os.path.realpath('.')+'/Environment/models/'+model_name+'/'+model_name+'.sdf',mode='w') as file:
            file.writelines(data)
            
    def model_list(self):
        """Simply splits the list of models

        Returns:
            [list]: list of models manually assigned.
        """        
        all_models = self.models.split(',')
        return all_models
    
def mesh_extractor(model):
    """Extracts mesh from the model folder.
    """    
    # model_directory = os.path.dirname(__file__)+'/Environment/models/'+ model +'/meshes/'+ model +'.stl'
    conf = Configuration()
    model_directory = conf.model_dir+'/'+ model +'/meshes/'+ model +'.stl'
    model_mesh = mesh.Mesh.from_file(model_directory)
    return model_mesh
    
def find_mins_maxs(obj):
    """This extracts the minimum and maximum value of the meshes boundaries.

    Args:
        obj 
        class.obj: A class object obtained from the numpy-stl library. The 
        functionality is directly tied to the mesh_extracto function.
        

    Returns:
        minx, maxx, miny, maxy, minz, maxz 
        float: The values of the cartesian coordinate.
    """   
    minx = obj.x.min()
    maxx = obj.x.max()
    miny = obj.y.min()
    maxy = obj.y.max()
    minz = obj.z.min()
    maxz = obj.z.max()
    return minx, maxx, miny, maxy, minz, maxz

def bound_box(model_name):
    """Generates bounding box for a given model.
    """    
    mesh_obj = mesh_extractor(model_name)
    cartesian_min_max_vals = find_mins_maxs(mesh_obj)
    adjusting_for_scaling = np.asarray(cartesian_min_max_vals)*0.001
    return adjusting_for_scaling   

def stl_property_extractor(model_name):
    """Views the property of a specific mesh, doesnt work well.
    """    
    meshy = mesh_extractor(model_name)
    volume, cog, inertia = meshy.get_mass_properties()
    print("Volume                                  = {0}".format(volume))
    print("Position of the center of gravity (COG) = {0}".format(cog))
    print("Inertia matrix at expressed at the COG  = {0}".format(inertia[0,:]))
    print("                                          {0}".format(inertia[1,:]))
    print("                                          {0}".format(inertia[2,:]))
    print(inertia/volume)
    

def choices(population, weights=None, *, cum_weights=None, k=1):
    """Return a k sized list of population elements chosen with replacement.
    If the relative weights or cumulative weights are not specified,
    the selections are made with equal probability.
    """
    n = len(population)
    if cum_weights is None:
        if weights is None:
            _int = int
            n += 0.0    # convert to float for a small speed improvement
            return [population[_int(random.random() * n)] for i in _repeat(None, k)]
        cum_weights = list(_accumulate(weights))
    elif weights is not None:
        raise TypeError('Cannot specify both weights and cumulative weights')
    if len(cum_weights) != n:
        raise ValueError('The number of weights does not match the population')
    bisect = _bisect
    total = cum_weights[-1] + 0.0   # convert to float
    hi = n - 1
    return [population[bisect(cum_weights, random.random() * total, 0, hi)]
            for i in _repeat(None, k)]
    
def collision_checker(model,x,y,z,model_tracer):
    """Checks the collision with all other dynamic models. Uses 
    Axis-Aligned Bounding Box.

    Args:
        obj_small_x (float): Model's bounding box minimum x coordinate.
        obj_big_x (float): Model's bounding box maximum x coordinate.
        obj_small_y (float): Model's bounding box minimum y coordinate.
        obj_big_y (float): Model's bounding box maximum y coordinate.

    Returns:
        bool: Returns True if there is no valid position for given model.
              Returns False if there is a valid position for given model.
    """    
    # Model to be placed bounding box
    org_sx = np.around(model.bounding_box[0]+x,2)+0.6
    org_bx = np.around(model.bounding_box[1]+x,2)+0.6
    org_sy = np.around(model.bounding_box[2]+y,2)+0.6
    org_by = np.around(model.bounding_box[3]+y,2)+0.6
    org_sz = np.around(model.bounding_box[4]+z,2)+0.6
    org_bz = np.around(model.bounding_box[5]+z,2)+0.6
        
    if model_tracer != []: 
             
        for models in np.arange(len(model_tracer)): 
            # Previous models bounding box
            com_sx = np.around(model_tracer[models].bounding_box[0]+float(model_tracer[models].x_coord),2)+0.6
            com_bx = np.around(model_tracer[models].bounding_box[1]+float(model_tracer[models].x_coord),2)+0.6
            com_sy = np.around(model_tracer[models].bounding_box[2]+float(model_tracer[models].y_coord),2)+0.6
            com_by = np.around(model_tracer[models].bounding_box[3]+float(model_tracer[models].y_coord),2)+0.6
            com_sz = np.around(model_tracer[models].bounding_box[4]+float(model_tracer[models].z_coord),2)+0.6
            com_bz = np.around(model_tracer[models].bounding_box[5]+float(model_tracer[models].z_coord),2)+0.6
            
            # Collision X-Y-Z-element
            if (org_sx < com_bx and org_bx > com_sx and org_sy < com_by and org_by > com_sy and org_sz < com_bz and org_bz > com_sz):
                return True
            # Collision X-Y-element Axis Aligned Collision Box
            # if (org_sx < com_bx and org_bx > com_sx and org_sy < com_by and org_by > com_sy):
            #     return True
        return False
    else:
        return False   
    
def navi_action_client(location):
    """Action client test for navigation using map.

    Args:
        loc (str): The location it has to go, updated in
        mas_domestic_robotics > mdr_environments > navigation goals.yaml

    Returns:
        bool: Returns boolean rarely works in my laptop.
    """    
    rospy.init_node('mdr_move_base_client_test')
    client = actionlib.SimpleActionClient('move_base_server', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()

    try:
        goal.goal_type = MoveBaseGoal.NAMED_TARGET
        goal.destination_location = location
        timeout = 500.0
        rospy.loginfo('Sending action lib goal to move_base_server, ' +
                    'destination : ' + goal.destination_location)
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
        print(client.get_result())
    except Exception as exc:
        print(str(exc))
        return False
    return True

def pose_action_client(coord_x, coord_y, direction):
    """Action client test for navigation using coordinates.

    Args:
        coord_x (float 32): x coordinate in the map for the robot to move towards.
        coord_y (float 32): y coordinate in the map for the robot to move towards.
        direction (float 32)(Degrees): Direction the robot should face.

    Returns:
        bool: Returns boolean rarely works in a laptop.
    """
    # rospy.init_node('mdr_move_base_client_test')
    client = actionlib.SimpleActionClient('move_base_server', MoveBaseAction)
    rospy.sleep(1)
    head = std_msgs.msg.Header(frame_id='map',stamp=rospy.Time.now())
    location = Point(x=coord_x, y=coord_y, z=0)
    R = 0
    P = 0
    Y = np.deg2rad(direction)
    quat1     = np.sin(R/2) * np.cos(P/2) * np.cos(Y/2) - np.cos(R/2) * np.sin(P/2) * np.sin(Y/2)
    quat2     = np.cos(R/2) * np.sin(P/2) * np.cos(Y/2) + np.sin(R/2) * np.cos(P/2) * np.sin(Y/2)
    quat3     = np.cos(R/2) * np.cos(P/2) * np.sin(Y/2) - np.sin(R/2) * np.sin(P/2) * np.cos(Y/2)
    quat4     = np.cos(R/2) * np.cos(P/2) * np.cos(Y/2) + np.sin(R/2) * np.sin(P/2) * np.sin(Y/2)
    angle = Quaternion(x=quat1,y=quat2,z=quat3,w=quat4)
        
    client.wait_for_server()
    goal = MoveBaseGoal()    
    try:
        goal.goal_type = MoveBaseGoal.POSE    
        goal.pose = PoseStamped(header=head, pose=Pose(position=location, orientation=angle))       
        timeout = 500.0
        rospy.loginfo('Sending action lib goal to move_base_server, ' + goal.destination_location)
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
        print(client.get_result())
    except Exception as exc:
        print(str(exc))
        return False
    return True