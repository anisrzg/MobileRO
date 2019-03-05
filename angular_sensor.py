#-------------------------------------------------------------------------------
# Name:        module1
# Purpose:
#
# Author:      Solange
#
# Created:     05/01/2019
# Copyright:   (c) Solange 2019
# Licence:     <your licence>
#-------------------------------------------------------------------------------

from lis3mdl import*
from math import*
import time
class Angular():
    offsetX = -150.0
    offsetY = -4000.0
    coeff_corr_Y = 1.0
    def __init__(self):
        self.set_parameters()
        self.magnet = LIS3MDL()
        self.magnet.enableLIS()
        time.sleep(0.5)
        self.origin_angle = self.get_angle()


    def get_angle(self):
        raw_vector=self.get_raw_vector()
        vector=[raw_vector[0] - self.offsetX , self.coeff_corr_Y*(raw_vector[1]-self.offsetY)]
        print(raw_vector)
        print(vector)
        if vector[0] > 0 :
            angle = ((360.0/(2.0*pi))*asin(float(vector[1])/float(sqrt(vector[0]**2+vector[1]**2))))
        else:
            if vector[1] > 0 : 
                angle = 180-((360.0/(2.0*pi))*asin(float(vector[1])/float(sqrt(vector[0]**2+vector[1]**2))))  
            else:
                angle = -180-((360.0/(2.0*pi))*asin(float(vector[1])/float(sqrt(vector[0]**2+vector[1]**2))))
        print(angle)
        return(angle)
    
    def get_error(self):
        return(self.origin_angle-self.get_angle())

    def get_origin_angle(self):
        return(self.origin_angle)
    
    def get_raw_vector(self):
        return(self.magnet.getMagnetometerRaw())
    
    def set_parameters(self):
        parameters=[]
        file = open("config.conf","r")
        for line in file.readlines():
            parameters.append(float(line))
        file.close()
        print("parameters loaded : ",parameters)
        self.offsetX = parameters[0]
        self.offsetY = parameters[1]
        self.coeff_corr_Y = parameters[2]
