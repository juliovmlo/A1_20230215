# -*- coding: utf-8 -*-
"""
Created on Wed Feb 15 09:56:20 2023

@author: julio
"""
import numpy as np
import matplotlib.pyplot as plt 
import classes
import functions as f

#Inicialize objects
WT = classes.WT()
Wind = classes.Wind()
Config = classes.Config()

# cl, cd, cm = f.force_coeffs_WT (WT, np.radians(0), 8)
# print(cl)

# V_0 = [0, 0,9]

# Wy_new, Wz_new = f.QuasySteadyIW(WT, Wind, V_0, 0.5, 0.5, np.radians(0), 10)
# print([0, 0, V_0])
# print(Wy_new, Wz_new)

# Initialization
deltaT = 0.1 #s

rotor_ang_lst = []
rotor_ang = 0

TotalRev = 1
t_end = 2*np.pi / WT.w * TotalRev
t_arr = np.arange(0,t_end, deltaT)

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
power_lst = []

for j, t in enumerate(t_arr):
    #Update of rotor position
    rotor_ang += WT.w*deltaT
    rotor_ang_lst.append(rotor_ang)
    
    #Pitch change
    change = 0
    if t > 5:
        WT.pitch_ang = np.radians(8)
    
    #Inicialize
    thrust = 0
    power = 0
    for i, b in enumerate(range(WT.B)):
        #Position of blade
        blade_ang = rotor_ang + b*2*np.pi/WT.B
        
        #Inicialize
        load_py_lst = []
        load_pz_lst = []
        for element in range(len(WT.r_lst)-1):
            pos = f.get_position(WT, blade_ang, element)
            px_lst.append(pos[0])
            py_lst.append(pos[1])
            
            #V_0 in the blade element
            V_0 = f.get_v0 (WT, Wind, Config, pos)
            
            Wy_new, Wz_new, py, pz, C_l, C_d = f.QuasySteadyIW(WT, Wind, V_0, Wy_old, Wz_old, blade_ang, element)
            Wy_old, Wz_old = Wy_new, Wz_new
            
            load_py_lst.append(py)
            load_pz_lst.append(pz)
            
            #Saving info of 11th element of the 1st blade
            if element == 10 and b == 0:
                V_y_lst.append(V_0[1])
                V_z_lst.append(V_0[2])
                C_l_lst.append(C_l)
                C_d_lst.append(C_d)
                load_py_elem_lst.append(py)
                load_pz_elem_lst.append(pz)
                Wy_lst.append(Wy_new)
                Wz_lst.append(Wz_new)
                
        #Last element has no loads
        load_py_lst.append(0)
        load_pz_lst.append(0)
        
        thrust += np.trapz(load_pz_lst, WT.r_lst) #Add each blade thrust
        py_times_r = [load_py_lst[i]*WT.r_lst[i] for i in range(len(WT.r_lst))]
        power += WT.w * np.trapz(py_times_r, WT.r_lst)
    thrust_lst.append(thrust)
    power_lst.append(power)

    
    
plt.figure(dpi=120)
plt.title('Load')
plt.plot(np.degrees(rotor_ang_lst[:]), load_py_elem_lst[:], label = 'py')
plt.plot(np.degrees(rotor_ang_lst[:]), load_pz_elem_lst[:], label = 'pz')
plt.ylabel('')
plt.legend()

plt.figure(dpi=120)
plt.title('Induced wind')
plt.plot(np.degrees(rotor_ang_lst[:]), Wy_lst[:], label = 'Wy')
plt.plot(np.degrees(rotor_ang_lst[:]), Wz_lst[:], label = 'Wz')
plt.ylabel('')
plt.legend()
plt.figure()

plt.figure(dpi=120)
plt.title('Aerodynamic coefficients')
plt.plot(np.degrees(rotor_ang_lst[:]), C_l_lst[:], label = 'Cl')
plt.plot(np.degrees(rotor_ang_lst[:]), C_d_lst[:], label = 'Cd')
plt.ylabel('')
plt.legend()
plt.figure()

plt.figure(dpi=120)
plt.title('Thrust')
plt.plot(t_arr[:], thrust_lst[:])
plt.ylabel('')
plt.figure()

plt.figure(dpi=120)
plt.title('Power')
plt.plot(t_arr[:], [power_lst[i]/1e6 for i in range(len(power_lst))])
plt.gca().set_ylim(bottom=0)
plt.ylabel('')
plt.figure()