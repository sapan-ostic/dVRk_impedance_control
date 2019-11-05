## The following snippet code is Written by Ajith Kumar

import yaml 
import numpy as np

## Loading the file
def yaml_loader(filepath):
    """ Loads a file path """
    with open(filepath, 'r') as file_descriptor:
        data = yaml.load(file_descriptor)
    return data

## Dumping it
def yaml_dumper(filepath):
    """ Dumps data to a yaml file"""
    with open (filepath, 'w') as file_descriptor:
        yaml.dump(data, file_descriptor)
               
# File path
file_path = "/home/aimlabx/ambf/ambf_models/descriptions/multi-bodies/robots/blender-kuka2.yaml"
data = yaml_loader(file_path)

# Bodies and Joints count
Bodies = []
Joints = []

def Bodies_count(data):
    bodies = data.get('bodies')
    
    for ele in bodies:
        Bodies.append(ele)
        
    num_of_bodies = len(bodies)
    
    return num_of_bodies, Bodies

def Joint_count(data):
    joints = data.get('joints')
    for ele in joints:
        Joints.append(ele)
        
    num_of_joints = len(joints)    
        
    return num_of_joints, Joints

# Joint count testing
n_j,j = Joint_count(data)

# Body Count Testing 
n_b, b = Bodies_count(data)

# Mass array 
def get_mass_array(data,Bodies):
    mass_arr=[]
    for body in Bodies:
        mass_arr.append(data[body]['mass'])
    return np.array(mass_arr)

# Testing get_mass_array
print(get_mass_array(data,Bodies))

# Location --> orientation array
def get_location_array(data,Bodies):
    orient_arr=[]
    pos_arr = []
    for body in Bodies:
        orient_temp_arr=[]
        orient_temp_arr.append(data[body]['location']['orientation']['r'])
        orient_temp_arr.append(data[body]['location']['orientation']['p'])
        orient_temp_arr.append(data[body]['location']['orientation']['y'])
        orient_arr.append(orient_temp_arr)
        
        pos_temp_arr = []
        pos_temp_arr.append(data[body]['location']['position']['x'])
        pos_temp_arr.append(data[body]['location']['position']['y'])
        pos_temp_arr.append(data[body]['location']['position']['z'])
        pos_arr.append(pos_temp_arr)
    return np.array(orient_arr), np.array(pos_arr)

# testing get_location_array
print(get_location_array(data,Bodies))        

def get_inertial_offset(data, Bodies):
    
    inn_off_arr = []
    for body in Bodies:
        inn_off_temp=[]
        inn_off_temp.append(data[body]['inertial offset']['position']['x'])
        inn_off_temp.append(data[body]['inertial offset']['position']['y'])
        inn_off_temp.append(data[body]['inertial offset']['position']['z'])
        inn_off_arr.append(inn_off_temp)
    
    return np.array(inn_off_arr)

#Test Inertial offset
print(get_inertial_offset(data,Bodies))


def get_joint_type(data, Joints):
    J_type = []
    for Joint in Joints:
        J_temp = []
        J_temp.append(data[Joint]['type'])
        J_type.append(J_temp)
     
    return np.array(J_type)

# Testing the get_joint_type function
print(get_joint_type(data, Joints))


def get_parent_pivot(data, Joints):
	pivot_type = []

	for Joint in Joints:
		pivot_temp_type=[]
		pivot_temp_type.append(data[Joint]['parent pivot']['x']) 
		pivot_temp_type.append(data[Joint]['parent pivot']['y']) 
		pivot_temp_type.append(data[Joint]['parent pivot']['z']) 
		pivot_type.append(pivot_temp_type)
	return np.array(pivot_type) 

print(get_parent_pivot(data,Joints))

ma = get_mass_array(data, Bodies)
print ma
com_val = np.array(
        [[0.001, 0., 0.06], [0., -0.017, 0.134], [0.0, -0.074, 0.009], [0.0, 0.017, 0.134], [-0.001, 0.081, 0.008],
         [0.0, -0.017, 0.129], [0.0, 0.07, 0.068], [0.006, 0., 0.015]])
print com_val
