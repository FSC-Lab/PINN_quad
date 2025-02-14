"""
Copyright (C) 2024  Longhao Qian

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""

import os
import sys
import utils.log_scan as ls
import utils.gen_test_case as gt
import subprocess
import ctypes
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

if __name__ == "__main__":
    dir = os.path.dirname(os.path.abspath(sys.argv[0])) + "/../build_release/src/quadrotor_control/"
    resultName = "ccm_result"
    stateName, numOfStates = ls.ScanLogInfo(dir + resultName + ".txt")
    data = ls.ScanBinaryLog(dir + resultName+ ".dat",
                            numOfStates + 1,
                            'd',
                            ctypes.sizeof(ctypes.c_double))
    t_array = np.array(data[stateName['t']])
    xIx = np.array(data[stateName['xI_0_0']])
    xIy = np.array(data[stateName['xI_1_0']])
    xIz = np.array(data[stateName['xI_2_0']])
    
    vIx = np.array(data[stateName['vI_0_0']])
    vIy = np.array(data[stateName['vI_1_0']])
    vIz = np.array(data[stateName['vI_2_0']])
    
    accIx = np.array(data[stateName['acc_0_0']])
    accIy = np.array(data[stateName['acc_1_0']])
    accIz = np.array(data[stateName['acc_2_0']])
    
    omegaDox = np.array(data[stateName['omegaDot_0_0']])
    omegaDoy = np.array(data[stateName['omegaDot_1_0']])
    omegaDoz = np.array(data[stateName['omegaDot_2_0']])

    v = np.zeros((len(vIx),))
    for i in range(len(vIx)):
        v[i] = np.sqrt(vIx[i]**2 + vIy[i]**2 + vIz[i]**2)
    
    phi = np.array(data[stateName['euler_0_0']])
    theta = np.array(data[stateName['euler_1_0']])
    psi = np.array(data[stateName['euler_2_0']])
    
    plt.figure(1)
    plt.plot(t_array, xIx)
    plt.plot(t_array, xIy)
    plt.plot(t_array, xIz)
    plt.ylabel('xI(m)')
    plt.xlabel('t(s)')
    plt.legend(['xIx', 'xIy', 'xIz'])
    plt.grid(True)

    plt.figure(2)
    plt.plot(t_array, vIx)
    plt.plot(t_array, vIy)
    plt.plot(t_array, vIz)
    plt.ylabel('vI(m)')
    plt.xlabel('t(s)')
    plt.legend(['vIx', 'vIy', 'vIz'])
    plt.grid(True)
    
    plt.figure(3)
    plt.plot(t_array, v)
    plt.ylabel('v(m/s)')
    plt.xlabel('t(s)')
    plt.grid(True)
 

    plt.figure(4)
    plt.plot(t_array, phi)
    plt.plot(t_array, theta)
    plt.plot(t_array, psi)
    plt.ylabel('euler(rad)')
    plt.xlabel('t(s)')
    plt.legend(['phi', 'theta', 'psi'])
    plt.grid(True)

    plt.figure(5)
    plt.plot(t_array, accIx)
    plt.plot(t_array, accIy)
    plt.plot(t_array, accIz)
    plt.ylabel('acc(m/s2)')
    plt.xlabel('t(s)')
    plt.legend(['accx', 'accy', 'accz'])
    plt.grid(True)

    plt.figure(6)
    plt.plot(t_array, omegaDox)
    plt.plot(t_array, omegaDoy)
    plt.plot(t_array, omegaDoz)
    plt.ylabel('omegaDot (rad/s2)')
    plt.xlabel('t(s)')
    plt.legend(['omegaDox', 'omegaDoy', 'omegaDoz'])
    plt.grid(True)

    fig = plt.figure(7)
    ax = fig.gca(projection='3d')
    ax.plot3D(xIx, xIy, xIz)
    ax.set_xlabel('x(m)')
    ax.set_ylabel('y(m)')
    ax.set_zlabel('z(m)')
    ax.set_box_aspect([1.0, 1.0, 1.0])
    plt.grid(True)
    
    plt.show()