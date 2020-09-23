#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
---------------------------------------------------- 
Random Scenario Generator

The RSG will be used to generate unique and altered
poses for the models that will be used within the
worlds of Gazebo simulator.
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
import subprocess
import numpy as np

import rospy
import std_srvs.srv as std_srvs
import gazebo_msgs.srv as gazebo_srvs
from geometry_msgs.msg import *
from gazebo_msgs.srv import SpawnModel

from utilities.utility import bound_box
from utilities.utility import choices
from termcolor import colored

global model_tracer
   
class World():
    """
    This class contains all relevant information of a gazebo world.
    """    
    def __init__(self):
        pass
    
    def world_properties(self):
        try:
            print(colored('Getting world properties.','yellow'))
            world_node = subprocess.Popen(['rosservice','call','gazebo/get_world_properties'])
        finally:
            time.sleep(2)
            world_node.terminate()     
    
    def reset_world(self):
        """
        This class contains resets the models poses.
        """    
        try:
            print(colored('Resetting model poses.','yellow'))
            world_node = subprocess.Popen(['rosservice','call','gazebo/reset_world'])
        finally:
            time.sleep(2)
            world_node.terminate() 
            
    def reset_simulation(self):
        """
        This class contains resets everything in gazebo.
        """    
        try:
            print(colored('Resetting everything.','yellow'))
            world_node = subprocess.Popen(['rosservice','call','gazebo/reset_simulation'])
        finally:
            time.sleep(2)
            world_node.terminate()  
            
    def unpause_simulation(self):
        """
        This class un pauses the physics engine in gazebo.
        """    
        try:
            print(colored('Un-pausing physics engine.','green'))
            world_node = subprocess.Popen(['rosservice','call','gazebo/unpause_physics'])
        finally:
            time.sleep(2)
            world_node.terminate()  

class Model():
    """
    This class contains all relevant information of a gazebo model.
    """    
    def __init__(self, model_real_name):
        
        conf = Configuration()

        self.model_dir       = conf.model_dir
        self.model_real_name = model_real_name
        self.model_node_name = str(np.random.randint(0,999)) 
        self.x_coord   = 0  # X-coordinate
        self.y_coord   = 0  # Y-coordinate
        self.z_coord   = 0  # Z-coordinate
        self.R_coord   = 0  # Roll in radians
        self.P_coord   = 0  # Pitch in radians
        self.Y_coord   = 0  # Yaw in radians
        self.quat1     = 0  # quaternion 1
        self.quat2     = 0  # quaternion 2
        self.quat3     = 0  # quaternion 3
        self.quat4     = 0  # quaternion 4
        
        self.bounding_box = bound_box(self.model_real_name)
        
    def model_properties(self, model_name):
        """
        This function obtains a models property.
        
        Args:
            model_name (str): Name of the object and the object ID from which the model will be obtained.
        """
        model_desc = '{model_name: '+ model_name +'}'

        try:
            print(colored('Getting {0} model properties.'.format(model_name),'yellow'))
            property_node = subprocess.Popen(['rosservice', 'call', 'gazebo/get_model_properties', model_desc])
        finally:
            time.sleep(1.5)
            property_node.terminate()   
            print(colored('Successfully obtained {0} model properties.'.format(model_name),'green')) 
               
    def insert_model(self):
        """
        This function inserts a model in the gazebo world by passing a roscommand from the terminal. 
        """        
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        try:
            print(colored('Inserting {0} model'.format(self.model_real_name),'yellow'))
            
            # Un-comment if first time using the package. Corrects the uri of sdf models.
            conf.model_uri_correcter(self.model_real_name)
            
            model_node = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
            model_node(model_name= self.model_real_name + self.model_node_name, 
                       model_xml = open(self.model_dir +'/'+self.model_real_name+'/'+self.model_real_name+'.sdf', 'r').read(), 
                       robot_namespace = "/RSG",
                       initial_pose = Pose(position=Point(self.x_coord,self.y_coord,self.z_coord),
                                          orientation=Quaternion(self.quat1,self.quat2,self.quat3,self.quat4)),
                       reference_frame ="world")
            print(colored('Successfully spawned {0} model'.format(self.model_real_name),'green'))
            
        except rospy.ServiceException as e:
            print(colored('Cannot spawn {0} model'.format(self.model_real_name),'red'))

        # Old version
        # try:
        #     print(colored('Inserting {0} model'.format(self.model_real_name),'yellow'))
            
        #     # Un-comment if first time using the package. Corrects the uri of sdf models.
        #     # conf.model_uri_correcter(self.model_real_name)
            
        #     model_node = subprocess.Popen(['rosrun','gazebo_ros','spawn_model',
        #     '-file',  self.model_dir +'/'+self.model_real_name+'/'+self.model_real_name+'.sdf','-sdf',
        #     '-model', self.model_real_name + self.model_node_name,
        #     '-reference_frame','world',
        #     '-x',self.x_coord ,
        #     '-y',self.y_coord ,
        #     '-z',self.z_coord ,
        #     '-R',self.R_coord ,
        #     '-P',self.P_coord ,
        #     '-Y',self.Y_coord ])
        # finally:
        #     time.sleep(2)
        #     model_node.terminate()   
        #     print(colored('Successfully spawned {0} model'.format(self.model_real_name),'green'))   

    def delete_model(self, model_name):
        """
        This function deletes a model in the gazebo world by passing a roscommand from the terminal.
        
        Args:
            model_name (str): Name of the object and the object ID to be deleted.
        """
        model_desc = '{model_name: '+ model_name +'}'

        try:
            print(colored('Deleting {0} model'.format(model_name),'yellow'))
            delete_node = subprocess.Popen(['rosservice', 'call', 'gazebo/delete_model', model_desc])
        finally:
            time.sleep(2)
            delete_node.terminate()   
            print(colored('Successfully deleted {0} model'.format(model_name),'green'))   

class Configuration():

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
        all_models = self.models.split(',')
        return all_models
           
def collision_checker(model,x,y,z):
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
    # original bounding box
    org_sx = np.around(model.bounding_box[0]+x,2)
    org_bx = np.around(model.bounding_box[1]+x,2)
    org_sy = np.around(model.bounding_box[2]+y,2)
    org_by = np.around(model.bounding_box[3]+y,2)
    org_sz = np.around(model.bounding_box[4]+z,2)
    org_bz = np.around(model.bounding_box[5]+z,2)
        
    if model_tracer != []: 
             
        for models in np.arange(len(model_tracer)): 
            # compared bounding box
            com_sx = np.around(model_tracer[models].bounding_box[0]+float(model_tracer[models].x_coord),2)
            com_bx = np.around(model_tracer[models].bounding_box[1]+float(model_tracer[models].x_coord),2)
            com_sy = np.around(model_tracer[models].bounding_box[2]+float(model_tracer[models].y_coord),2)
            com_by = np.around(model_tracer[models].bounding_box[3]+float(model_tracer[models].y_coord),2)
            com_sz = np.around(model_tracer[models].bounding_box[4]+float(model_tracer[models].z_coord),2)
            com_bz = np.around(model_tracer[models].bounding_box[5]+float(model_tracer[models].z_coord),2)
            
            # Collision X-Y-Z-element
            if (org_sx < com_bx and org_bx > com_sx and org_sy < com_by and org_by > com_sy and org_sz < com_bz and org_bz > com_sz):
                return True
            # Collision X-Y-element
            # if (org_sx < com_bx and org_bx > com_sx and org_sy < com_by and org_by > com_sy):
            #     return True

        return False
    else:
        return False
        
           
if __name__ == "__main__":
    
    conf = Configuration()
    world = World()
    model_tracer = []

 
    print(colored('Starting HSR Simulator', 'green'))
    try:
        hsr_node = subprocess.Popen(['roslaunch', conf.rospakg, conf.ros_file_name])
        time.sleep(10)
        
        # Static model placement
        static_model = Model('table')
        static_model.x_coord, static_model.y_coord, static_model.z_coord = '0','4','0'

        # Dynamic model placement
        model_list = choices(conf.model_list(), k=int(conf.num_of_mod))
        
        # Ensuring the models spawns in the vicinity of the static model in this case the table
        minx = np.around(static_model.bounding_box[0]*0.8+float(static_model.x_coord),2)
        maxx = np.around(static_model.bounding_box[1]*0.8+float(static_model.x_coord),2)
        miny = np.around(static_model.bounding_box[2]*0.8+float(static_model.y_coord),2)
        maxy = np.around(static_model.bounding_box[3]*0.8+float(static_model.y_coord),2)
        minz = np.around(static_model.bounding_box[4]+float(static_model.z_coord),2)
        maxz = np.around(static_model.bounding_box[5]+float(static_model.z_coord),2)

        for model in model_list:
            dynamic_model = Model(model)
                                    
            # Ensuring models don't spawn on top of existing models
            for iteration in range(0,1000):
                
                x = random.uniform(minx, maxx)
                y = random.uniform(miny, maxy)
                z = maxz + 0.03

                check = collision_checker(dynamic_model,x,y,z)
                if check == False:
                    
                    dynamic_model.x_coord = x
                    dynamic_model.y_coord = y
                    dynamic_model.z_coord = z
                    
                    dynamic_model.insert_model()
                        
                    model_tracer.append(dynamic_model)
                    break
                
                if iteration == 999:
                    print(colored('No space for model insertion ','red'),dynamic_model.model_real_name)
                    continue
                
        print(colored('Completed spawning dynamic models','cyan'))
                
        # # Get model properties
        # static_model.model_properties(model_tracer[1].model_real_name + model_tracer[1].model_node_name)
        # # Delete a model
        # static_model.delete_model(model_tracer[1].model_real_name + model_tracer[1].model_node_name)
        
        # world.unpause_simulation()

    finally:
        time.sleep(6000)
        hsr_node.terminate()  
        print(colored('Terminating ros!','red'))    