# -*- coding: utf-8 -*-
"""
Created on Wed Feb 15 10:38:38 2023

@author: julio
"""
import numpy as np

class WT:#Wind Turbine 
    def __init__(self):
            #Load blade data
            bladedata = np.loadtxt('airfoildata/bladedat.txt', skiprows=0)
            r_col = 0
            b_col = 1 #twist angle
            c_col = 2 #chord
            t_col = 3 #thickness
            #Changes beta from degrees to radians
            bladedata [:,b_col] = np.radians(bladedata[:, b_col])
            self.r_lst = bladedata [:,r_col]
            self.b_lst = bladedata [:,b_col]
            self.c_lst = bladedata [:,c_col]
            self.t_lst = bladedata [:,t_col]
            
            #Load airfoil data
            # NOTE THAT IN PYTHON THE INTERPOLATION REQUIRES THAT THE VALUES INCREASE
            #IN THE VECTOR! 
            self.thick_lst = [24.1, 30.1, 36, 48, 60, 100]
            files = ['airfoildata/FFA-W3-241.txt','airfoildata/FFA-W3-301.txt',
                   'airfoildata/FFA-W3-360.txt','airfoildata/FFA-W3-480.txt',
                   'airfoildata/FFA-W3-600.txt','airfoildata/cylinder.txt']
            
            #Readin of tables:
            self.cl_tab = np.zeros([105,6])
            self.cd_tab = np.zeros([105,6])
            self.cm_tab = np.zeros([105,6])
            for i in range(np.size(files)):
             self.attack_ang_lst, self.cl_tab[:,i], self.cd_tab[:,i], self.cm_tab[:,i] = np.loadtxt(files[i], skiprows=0).T
            
            return
        
    #Geometry
    H = 119   #[m]
    L_s = 7.1 #[m]
    R = 89.15 #[m]
    B = 3 #Num. of blades
    tilt_ang = np.radians(0) #degrees
    cone_ang = np.radians(0) #degrees
    a = 3.32 # Tower radius [m]

    #Operation
    n = 7.224 #RPM
    w = n * np.pi / 60 #rad/s
    yaw_ang = np.radians(0) #degrees
    pitch_ang = np.radians(0) #degrees
    
class Wind:#Wind conditions
    V_0_H = 10 #m/s
    nu = 0.2 #shear
    rho = 1.225 #density [kg/m3]
    
class Config:
    WindShear = True
    TowerEffect = True