# -*- coding: utf-8 -*-
"""
Created on Wed Feb 15 10:40:47 2023

@author: julio
"""
import numpy as np
from numpy import linalg as LA

def get_R12 (WT):
    #R matrices
    R_tilt = np.array([[np.cos(WT.tilt_ang), 0, -np.sin(WT.tilt_ang)],
                    [0,                1,                 0],
                    [np.sin(WT.tilt_ang), 0, np.cos(WT.tilt_ang)]])
    R_yaw = np.array([[1,      0,                       0],
                    [0, np.cos(WT.yaw_ang),  np.sin(WT.yaw_ang)],
                    [0, -np.sin(WT.yaw_ang), np.cos(WT.yaw_ang)]])

    R12 = R_tilt @ R_yaw
    return R12

def get_R14 (WT, blade_ang):
    #R matrices
    R12 = get_R12(WT)
    
    #Update rotational matrices with current blade angle
    R23 = np.array([[np.cos(blade_ang),  np.sin(blade_ang), 0],
                    [-np.sin(blade_ang), np.cos(blade_ang), 0],
                    [0,                            0,         1]])

    R34 = np.array([[np.cos(WT.cone_ang), 0, -np.sin(WT.cone_ang)],
                    [0,                1,                 0],
                    [np.sin(WT.cone_ang), 0, np.cos(WT.cone_ang)]])

    #Operate rotational matrix from frame of reference 1 to 4
    R14 = R34 @ R23 @ R12
    
    return R14

def get_position (WT, blade_ang, element):
    """
    Parameters
    ----------
    WT : Class
        WindTurbina class consisting of geometry and operation parameters.
    r : Float
        Radial position. [m]
    blade_ang : Float
        Azimudal angle of the blade.[rad]

    Returns
    -------
    pos : 3 element array.
        Position in inertial coordinates of the blade element. [x,y,z] [m]
    """
    r = WT.r_lst[element]
    R21 = get_R12(WT).transpose()
    R14 = get_R14 (WT, blade_ang)
    #P vectors
    p_t = np.array([WT.H, 0, 0])
    p_s = R21 @ np.array([0, 0, -WT.L_s])
    p_b =  np.array([r, 0, 0])
    
    #Obtain absolute position of blade element
    pos = p_t + R21 @ p_s + R14.transpose() @ p_b
    
    return pos

def tower_model(WT, pos, V_0_z):
    """
    This function corrects the free wind speed to account for the tower.

    Parameters
    ----------
    WT : Class
        WindTurbina class consisting of geometry and operation parameters.
    p : 3 element array.
        Position in inertial coordinates of the blade element. [x,y,z] [m]
    V_0_z : Float
        Free wind speed.

    Returns
    -------
    V_0_TM : 3 element vector.
        Free wind speed correcting for the tower effect.

    """
    x = pos[0]
    if x > WT.H:
        V_0_TM = [0, 0, V_0_z]
    else:
        y = pos[1]; z = pos[2]
        r = np.sqrt(y**2 + z**2)
        V_r = z/r*V_0_z*(1 - (WT.a/r)**2)
        V_theta = y/r*V_0_z*(1 +(WT.a/r)**2)
        
        #Wind velocity in frame of reference 1
        V_0_TM = np.array([0,
                        y/r*V_r - z/r*V_theta,
                        z/r*V_r + y/r*V_theta])
        
    return V_0_TM

def get_v0 (WT, Wind, Config, pos):
    
    #Apply wind shear
    if(Config.WindShear): V_0 = [0, 0, Wind.V_0_H*(pos[0]/WT.H)**Wind.nu]
    else: V_0 = [0, 0, Wind.V_0_H]
    
    #Apply tower effect
    if(Config.TowerEffect): V_0 = tower_model(WT, pos, V_0[2])
    
    return V_0

def force_coeffs_WT(WT, Config, attack_ang, V_rel_abs):
    #Could try to make it simpler just having as an imput (WT, thickness, angle and V_rel)
    
    """Gets the force coefficients given the angle of attack and the
    thickness ratio.
    
    Keywords arguments:
        WT -- Wind turbine class
        attack_ang -- angle of attack [rad]
        i -- blade element [-]
    Returns:
        cl -- lift coefficinet [-]
        cd -- drag coefficient [-]
        cm -- pitch moment coeficient [-]
    """
    
    cl_stat_aoa = np.zeros([1,6])
    cd_stat_aoa = np.zeros([1,6])
    fs_stat_aoa = np.zeros([1,6])
    cl_inv__aoa = np.zeros([1,6])
    cl_fs_aoa = np.zeros([1,6])
    
    # self.cl_stat_tab = np.zeros([data_points,len(self.thick_lst)])
    # self.cd_stat_tab = np.zeros([data_points,6])
    # self.fs__stat_tab = np.zeros([data_points,6])
    # self.cl__inv_tab = np.zeros([data_points,6])
    # self.cl__fs_tab = np.zeros([data_points,6])
    
    #Interpolate to current angle of attack (aoa):
    #Ineficient: could interpolate only two closest tables
    for i in range(np.size(WT.thick_lst)):
        cl_stat_aoa[0,i] = np.interp (np.degrees(attack_ang), WT.attack_ang_lst, WT.cl_stat_tab[:,i])
        cd_stat_aoa[0,i] = np.interp (np.degrees(attack_ang), WT.attack_ang_lst, WT.cd_stat_tab[:,i])
        fs_stat_aoa[0,i] = np.interp (np.degrees(attack_ang), WT.attack_ang_lst, WT.fs__stat_tab[:,i])
        cl_inv__aoa[0,i] = np.interp (np.degrees(attack_ang), WT.attack_ang_lst, WT.cl__inv_tab[:,i])
        cl_fs_aoa[0,i] = np.interp (np.degrees(attack_ang), WT.attack_ang_lst, WT.cl__fs_tab[:,i])
    
    #Interpolate to current thickness:
    cl_stat = np.interp (WT.t_lst[WT.element], WT.thick_lst, cl_stat_aoa[0,:])
    C_d = np.interp (WT.t_lst[WT.element], WT.thick_lst, cd_stat_aoa[0,:])
    fs_stat = np.interp (WT.t_lst[WT.element], WT.thick_lst, fs_stat_aoa[0,:])
    cl_inv = np.interp (WT.t_lst[WT.element], WT.thick_lst, cl_inv__aoa[0,:])
    cl_fs = np.interp (WT.t_lst[WT.element], WT.thick_lst, cl_fs_aoa[0,:])
    
    if Config.DynStall:
        tau = 4 * WT.c_lst[WT.element] / V_rel_abs
        f_s = fs_stat + (WT.last_f_s[WT.element,WT.blade] - fs_stat)*np.exp(-Config.deltaT/tau)
        C_l = f_s*cl_inv + (1-f_s)*cl_fs
    else:
        C_l = cl_stat
    
    return C_l, C_d

def QuasySteadyIW(WT, Config, Wind, V_0, Wy_old, Wz_old, blade_ang, element):
    """
    Quasy Steady Induced Wind
    Calculates the quasi steady induced velocities, i.e. the velocities that
    satisfy the equilibrium with the aerodynamic loads.
    
    Parameters
    ----------
    WT : Class
        WindTurbina class consisting of geometry and operation parameters.
    
    """
    
    V_0_4 = get_R14(WT, blade_ang) @ V_0
    
    #Relative wind without updating induced wind
    V_rel_y = V_0_4[1] + Wy_old - WT.w*WT.r_lst[element]*np.cos(WT.cone_ang)
    V_rel_z = V_0_4[2] + Wz_old
    V_rel_abs = LA.norm(np.array([V_rel_y, V_rel_z]))
    
    #Getting the lift and  drag coefficients
    flow_ang = np.arctan(V_rel_z/-V_rel_y)
    twist_ang = WT.b_lst[element]
    attack_ang = flow_ang - (twist_ang + WT.pitch_ang)
    C_l, C_d = force_coeffs_WT(WT, Config, attack_ang, V_rel_abs)
                        
    #Glauert's correction
    a = -Wz_old / Wind.V_0_H #To get the induce factor we need to use the hub hight wind velocity
    if a > 1/3: f_g = 1/4*(5-3*a)
    else: f_g = 1
    
    #Plandtl's tip loss correction
    F = 2/np.pi * np.arccos(np.exp(-WT.B/2 * (WT.R - WT.r_lst[element])/(WT.r_lst[element]*np.sin(abs(flow_ang)))))
    
    #Getting the lift anf drag
    Const = 1/2 * Wind.rho * V_rel_abs**2 * WT.c_lst[element]
    lift = Const * C_l
    drag = Const * C_d
    
    py = lift*np.sin(flow_ang) - drag*np.cos(flow_ang)
    pz = lift*np.cos(flow_ang) + drag*np.sin(flow_ang)
    
    denom = 4*np.pi*Wind.rho*WT.r_lst[element]*F*np.sqrt(V_0_4[1]**2 + (V_0_4[2] + f_g*Wz_old)**2)
    Wy_qs = -WT.B*lift*np.sin(flow_ang)/denom
    Wz_qs = -WT.B*lift*np.cos(flow_ang)/denom
    
    return Wy_qs, Wz_qs, a, py, pz, C_l, C_d

def DynFiltering (Wy_qs, Wz_qs, a, V_0, element, b, WT, Config):
    if a > 0.5: a = 0.5
    k = 0.6
    tau1 = 1.1/(1-1.3*a)*WT.R/V_0
    tau2 = (0.39 - 0.26*(WT.r_lst[element]/WT.R)**2)*tau1
    
    for i, W_qs in enumerate([Wy_qs, Wz_qs]):
        
        H = W_qs + k * tau1*(W_qs - WT.last_W_qs[i,element,b])/Config.deltaT
        W_int = H + (WT.last_W_int[i,element,b] - H)*np.exp(-Config.deltaT/tau1)
        W = W_int + (WT.last_W[i,element,b] - W_int)*np.exp(-Config.deltaT/tau2)
        
        #Store
        WT.last_W_int[i,element,b] = W_int
        WT.last_W[i,element,b] = W
    
    if element == 11:   
        debug =True
    
    return WT.last_W_int[0,element,b], WT.last_W_int[1,element,b]