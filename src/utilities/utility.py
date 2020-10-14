#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
---------------------------------------------------- 
Utilities

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

import random
import sys,os
import numpy
from stl import mesh
from itertools import accumulate as _accumulate, repeat as _repeat
from bisect import bisect as _bisect

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
    
def mesh_extractor(model):
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
    mesh_obj = mesh_extractor(model_name)
    cartesian_min_max_vals = find_mins_maxs(mesh_obj)
    adjusting_for_scaling = numpy.asarray(cartesian_min_max_vals)*0.001
    return adjusting_for_scaling   

def stl_property_extractor(model_name):
    meshy = mesh_extractor(model_name)
    volume, cog, inertia = meshy.get_mass_properties()
    print("Volume                                  = {0}".format(volume))
    print("Position of the center of gravity (COG) = {0}".format(cog))
    print("Inertia matrix at expressed at the COG  = {0}".format(inertia[0,:]))
    print("                                          {0}".format(inertia[1,:]))
    print("                                          {0}".format(inertia[2,:]))
    print(inertia/volume)
    
# stl_property_extractor('knife')


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