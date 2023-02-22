# -*- coding: utf-8 -*-
"""
Created on Wed Feb 15 09:56:20 2023

@author: julio
"""
import numpy as np
from numpy import linalg as LA
import matplotlib.pyplot as plt 
import classes
import functions as f

#Inicialize objects
WT = classes.WT()
Wind = classes.Wind()
Config = classes.Config()

# Initialization
rotor_ang_lst = []
rotor_ang = 0

TotalRev = 2
t_end = 2*np.pi / WT.w * TotalRev
t_arr = np.arange(0,t_end, Config.deltaT)

px_lst = [] #Absolute position
py_lst = []

V_y_lst = []
V_z_lst = []

V_0_y_lst = []
V_0_z_lst = []

Wy_lst = []
Wz_lst = []

Wy_old = 0; Wz_old = 0
C_l_lst = []
C_d_lst = []

load_py_elem_lst = []
load_pz_elem_lst = []

thrust_lst = []
blade_thrust_tab = np.zeros((WT.B, len(t_arr)))
power_lst = []

for t_step, t in enumerate(t_arr):
    
    # #Pitch change
    # change = 0
    # if t > 5:
    #     WT.pitch_ang = np.radians(8)
    
    #Inicialize blade loop
    thrust = 0
    power = 0
    for i, b in enumerate(range(WT.B)):
        WT.blade = b
        
        #Update position of blade
        blade_ang = rotor_ang + b*2*np.pi/WT.B
        
        #Inicialize radius loop
        load_py_lst = []
        load_pz_lst = []
        for element in range(len(WT.r_lst)-1):
            WT.element = element
            
            #Calculate blade element position
            pos = f.get_position(WT, blade_ang, element)
            
            #V_0 in the blade element
            V_0_vec = f.get_v0 (WT, Wind, Config, pos)
            
            #Quasy Steady Induced Wind
            Wy_qs, Wz_qs, a, py, pz, C_l, C_d = f.QuasySteadyIW(WT, Config, Wind, V_0_vec, WT.last_W[0,element,b], WT.last_W[1,element,b], blade_ang, element)
            
            #Dynamic Filtering
            if Config.DynFilter:
                Wy_new, Wz_new = f.DynFiltering (WT.last_W[0,element,b], WT.last_W[1,element,b], a, LA.norm(V_0_vec), element, b, WT, Config)
            else:
                Wy_new, Wz_new = Wy_qs, Wz_qs
            
            WT.last_W[0,element,b] = Wy_new
            WT.last_W[1,element,b] = Wz_new
                       
            load_py_lst.append(py)
            load_pz_lst.append(pz)
            
            #Saving info of 11th element of the 1st blade
            if element == 8 and b == 0:
                px_lst.append(pos[0])
                py_lst.append(pos[1])
                V_y_lst.append(V_0_vec[1])
                V_z_lst.append(V_0_vec[2])
                C_l_lst.append(C_l)
                C_d_lst.append(C_d)
                load_py_elem_lst.append(py)
                load_pz_elem_lst.append(pz)
                Wy_lst.append(Wy_new)
                Wz_lst.append(Wz_new)
                
        #Last element has no loads
        load_py_lst.append(0)
        load_pz_lst.append(0)
        
        blade_thrust_tab[WT.blade, t_step] = np.trapz(load_pz_lst, WT.r_lst)
        thrust += np.trapz(load_pz_lst, WT.r_lst) #Add each blade thrust
        py_times_r = [load_py_lst[i]*WT.r_lst[i] for i in range(len(WT.r_lst))]
        power += WT.w * np.trapz(py_times_r, WT.r_lst)
    thrust_lst.append(thrust)
    power_lst.append(power)
    
    #Update of rotor position
    rotor_ang += WT.w*Config.deltaT
    rotor_ang_lst.append(rotor_ang)

#Plot

#Debug

#The interpolation function works
# WT.element = 4
# print(WT.t_lst[WT.element])
# print(f.force_coeffs_WT(WT, Config, np.radians(30), 0))

#position

# plt.figure(dpi=120)
# plt.title('Blade pos')
# plt.plot(np.degrees(rotor_ang_lst[:]), px_lst[:], label = 'px')
# plt.plot(np.degrees(rotor_ang_lst[:]), py_lst[:], label = 'px')
# plt.legend()
# plt.ylabel('[m]')
# plt.figure()

# plt.figure(dpi=120)
# plt.title('Local blade velocity')
# plt.plot(np.degrees(rotor_ang_lst[:]), V_y_lst[:], label = 'V_y')
# plt.plot(np.degrees(rotor_ang_lst[:]), V_z_lst[:], label = 'V_z')
# plt.legend()
# plt.ylabel('[m/s]')
# plt.figure()

# plt.figure(dpi=120)
# plt.title('Lift and drag')
# plt.plot(np.degrees(rotor_ang_lst[:]), C_l_lst[:], label = 'C_l')
# plt.plot(np.degrees(rotor_ang_lst[:]), C_d_lst[:], label = 'C_d')
# plt.legend()
# plt.ylabel('[m/s]')
# plt.figure()

# 1st
# """Config
# WindShear = False
# TowerEffect = False
# DynFilter = False
# DynStall = False
# """

# plt.figure(dpi=120)
# plt.title('Staedy state thrust force')
# plt.plot(t_arr[:], np.array(thrust_lst[:])/1e3)
# plt.gca().set_ylim(bottom=0)
# plt.ylabel('Thrust force [kN]')
# plt.figure()

# plt.figure(dpi=120)
# plt.title('Steady state power')
# plt.plot(t_arr[:], np.array(power_lst)/1e6)
# plt.gca().set_ylim(bottom=0)
# plt.ylabel('Power [kW]')
# plt.figure()

# plt.figure(dpi=120)
# plt.title('Normal and tangential load distributions in the blades')
# plt.plot(WT.r_lst[:], load_pz_lst[:], label = 'Normal load, p$_z$')
# plt.plot(WT.r_lst[:], load_py_lst[:], label = 'Tangential load, p$_y$')
# plt.gca().set_ylim(bottom=0)
# plt.legend()
# plt.ylabel('Load [N/m]')
# plt.xlabel('Radius [m]')
# plt.figure()

#2

plt.figure(dpi=120)
plt.title('Thrust force')
rev_lst = [t*WT.w/(2*np.pi) for t in t_arr]
plt.plot(rev_lst[:], np.array(thrust_lst[:])/1e3, label = 'T$_{Total}$')
plt.plot(rev_lst[:], np.array(blade_thrust_tab[0,:])/1e3, label = 'T$_{blade 1}$')
plt.plot(rev_lst[:], np.array(blade_thrust_tab[1,:])/1e3, label = 'T$_{blade 2}$')
plt.plot(rev_lst[:], np.array(blade_thrust_tab[2,:])/1e3, label = 'T$_{blade 3}$')
plt.gca().set_ylim(bottom=0)
plt.ylabel('Thrust force [kN]')
plt.xlabel('Revolutions [-]')
plt.legend()
plt.figure()

# fig = plt.figure(dpi=120)
# ax1 = fig.add_subplot(111)
# ax2 = ax1.twiny()
# plt.title('Thrust force')
# ax1.plot(t_arr[:], np.array(thrust_lst[:])/1e3, label = 'T$_{Total}$')
# ax1.plot(t_arr[:], np.array(blade_thrust_tab[0,:])/1e3, label = 'T$_{blade 1}$')
# ax1.plot(t_arr[:], np.array(blade_thrust_tab[1,:])/1e3, label = 'T$_{blade 2}$')
# ax1.plot(t_arr[:], np.array(blade_thrust_tab[2,:])/1e3, label = 'T$_{blade 3}$')
# # ax1.gca().set_ylim(bottom=0)
# plt.ylabel('Thrust force [kN]')
# plt.xlabel('Time [s]')
# ax1.legend()
# rev_lst = [t*WT.w/(2*np.pi) for t in t_arr]
# ax2.set_xlim(ax1.get_xlim())
# ax2.set_xticks(t_arr[:], rev_lst)
# plt.show()

#3

  
# plt.figure(dpi=120)
# plt.title('Load')
# plt.plot(np.degrees(rotor_ang_lst[:]), load_py_elem_lst[:], label = 'py')
# plt.plot(np.degrees(rotor_ang_lst[:]), load_pz_elem_lst[:], label = 'pz')
# plt.ylabel('')
# plt.legend()

# plt.figure(dpi=120)
# plt.title('Induced wind')
# plt.plot(np.degrees(rotor_ang_lst[:]), Wy_lst[:], label = 'Wy')
# plt.plot(np.degrees(rotor_ang_lst[:]), Wz_lst[:], label = 'Wz')
# plt.ylabel('')
# plt.legend()
# plt.figure()

# plt.figure(dpi=120)
# plt.title('Aerodynamic coefficients')
# plt.plot(np.degrees(rotor_ang_lst[:]), C_l_lst[:], label = 'Cl')
# plt.plot(np.degrees(rotor_ang_lst[:]), C_d_lst[:], label = 'Cd')
# plt.ylabel('')
# plt.legend()
# plt.figure()

# plt.figure(dpi=120)
# plt.title('Thrust')
# plt.plot(t_arr[:], thrust_lst[:])
# plt.ylabel('')
# plt.figure()

# plt.figure(dpi=120)
# plt.title('Power')
# plt.plot(t_arr[:], np.array(power_lst)/1e6)
# plt.gca().set_ylim(bottom=0)
# plt.ylabel('')
# plt.figure()