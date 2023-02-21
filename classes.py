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
            
            #Reading of tables:
            blade_ele = 105
            self.cl_tab = np.zeros([blade_ele,len(self.thick_lst)])
            self.cd_tab = np.zeros([blade_ele,6])
            self.cm_tab = np.zeros([blade_ele,6])
            for i in range(np.size(files)):
             self.attack_ang_lst, self.cl_tab[:,i], self.cd_tab[:,i], self.cm_tab[:,i] = np.loadtxt(files[i], skiprows=0).T
            
                      
            #Blade element information
            self.px = []*blade_ele
            self.py = []*blade_ele
            self.Wy_old = np.zeros([blade_ele, self.B])
            self.Wz_old = np.zeros([blade_ele, self.B])
            self.last_W = np.zeros([2,blade_ele, self.B])
            self.last_W_qs = np.zeros([2,blade_ele, self.B])
            self.last_W_int = np.zeros([2,blade_ele, self.B])
            return
        
    #Default Geometry
    H = 119   #[m]
    L_s = 7.1 #[m]
    R = 89.15 #[m]
    B = 3 #Num. of blades
    tilt_ang = np.radians(0) #degrees
    cone_ang = np.radians(0) #degrees
    a = 3.32 # Tower radius [m]

    #Default Operation
    n = 7.224 #RPM
    w = n * np.pi / 60 #rad/s
    yaw_ang = np.radians(0) #degrees
    pitch_ang = np.radians(0) #degrees
    

class Wind:#Wind conditions
    V_0_H = 10 #m/s
    nu = 0.2 #shear
    rho = 1.225 #density [kg/m3]
    
class Config:
    deltaT = 0.1
    WindShear = True
    TowerEffect = True
    DynFilter = True
    
class Sim(WT):#This class contains the current information of the simulation and stores it
    def __ini__(self):
        self.t_end = 2*np.pi / WT.w * self.TotalRev
        self.t_arr = np.arange(0, self.t_end, self.deltaT)
        
    #Default Configuration
    deltaT = 0.1 #[s]
    TotalRev = 1
    
    WindShear = True
    TowerEffect = True
    DynFilter = False
    
    #Blade element information buffer
    last_W = []*2
    last_W_qs = []*2
    last_W_int = []*2
    
    #Memory
    px = []*105
    py = []*105