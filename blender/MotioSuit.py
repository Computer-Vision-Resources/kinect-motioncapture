'''

Code to control the 3D model 

A server is setup to read the data from the processing client
The quaternion data is parsed from the input string 
The parsed data is transformed to match the axes of the model
The transformed data is then mapped to the individual parts of the model

'''



import bge
import math
from math import *
import mathutils
import time

import sys
import socket
import glob


HOST = ''
PORT = 5049

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)


s.bind((HOST,PORT))
s.listen(1)

conn, addr = s.accept()
print("Connected to ", addr)


# Get the whole bge scene
scene = bge.logic.getCurrentScene()
# Helper vars for convenience
source = scene.objects
# Get the whole Armature
main_arm = source.get('Armature')
ob = bge.logic.getCurrentController().owner


def updateAngles():
    data = conn.recv(1024).decode()
    joint = data.split()
    
    for x in joint:
        if x == '0':
            index = joint.index(x)
            trunk = mathutils.Quaternion((float(joint[index+1]),float(joint[index+2]),float(joint[index+3]),float(joint[index+4])))
            correction = mathutils.Quaternion((0.0, 1.0, 0.0), math.radians(180.0))    
            trunk_out = correction*trunk
        	
            ob.channels['trunk'].rotation_quaternion = trunk_out
    
        if x == '1':
            index = joint.index(x)
            
            #armR = mathutils.Quaternion((float(joint[index+1]),float(joint[index+2]),float(joint[index+3]),float(joint[index+4])))
     
        
        if x == '2':
            
            index = joint.index(x)
            
            #forearmR = mathutils.Quaternion((float(joint[index+1]),float(joint[index+2]),float(joint[index+3]),float(joint[index+4])))
            
            
        if x == '3':
            index = joint.index(x)
            
            armL = mathutils.Quaternion((float(joint[index+1]),float(joint[index+2]),float(joint[index+3]),float(joint[index+4])))
            correction = mathutils.Quaternion((0.0, 0.0, 1.0), math.radians(180.0))
            
            armL_out = correction*armL    
            #ob.channels['armL'].rotation_quaternion = armL_out             
    ob.update()
    time.sleep(0.001)

    


'''
	trunk = mathutils.Quaternion((angles[4][0],angles[4][1],angles[4][2],angles[4][3]))
	correction = mathutils.Quaternion((1.0, 0.0, 0.0), math.radians(90.0))
	trunk_out = correction*trunk

	upperLegR = mathutils.Quaternion((angles[5][0],angles[5][1],angles[5][2],angles[5][3]))
	correction = mathutils.Quaternion((1.0, 0.0, 0.0), math.radians(90.0))
	upperLegR_out = correction*upperLegR

	lowerLegR = mathutils.Quaternion((angles[6][0],angles[6][1],angles[6][2],angles[6][3]))
	correction = mathutils.Quaternion((1.0, 0.0, 0.0), math.radians(90.0))
	lowerLegR_out = correction*lowerLegR

	upperLegL = mathutils.Quaternion((angles[7][0],angles[7][1],angles[7][2],angles[7][3]))
	correction = mathutils.Quaternion((1.0, 0.0, 0.0), math.radians(90.0))
	upperLegL_out = correction*upperLegL

	lowerLegL = mathutils.Quaternion((angles[8][0],angles[8][1],angles[8][2],angles[8][3]))
	correction = mathutils.Quaternion((1.0, 0.0, 0.0), math.radians(90.0))
	lowerLegL_out = correction*lowerLegL

	ob.channels['armR'].rotation_quaternion = mathutils.Vector([angles[0][0],angles[0][1],angles[0][2],angles[0][3]])
	ob.channels['forearmR'].rotation_quaternion = mathutils.Vector([angles[1][0],angles[1][1],angles[1][2],angles[1][3]])
	ob.channels['armL'].rotation_quaternion = mathutils.Vector([angles[2][0],angles[2][1],angles[2][2],angles[2][3]])
	ob.channels['forearmL'].rotation_quaternion = mathutils.Vector([angles[3][0],angles[3][1],angles[3][2],angles[3][3]])
	ob.channels['trunk'].rotation_quaternion = trunk_out
	ob.channels['upperLegR'].rotation_quaternion = upperLegR_out
	ob.channels['lowerLegR'].rotation_quaternion = lowerLegR_out
	ob.channels['upperLegL'].rotation_quaternion = upperLegL_out
	ob.channels['lowerLegL'].rotation_quaternion = lowerLegL_out

	ob.update()
    
    time.sleep(0.001)

'''
