
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
import numpy as np
import struct
import ctypes
import os
import sys
import control as ctrl
import cvxpy as cp


def ReadDouble(content, idx):
    beginIdx = ctypes.sizeof(ctypes.c_double) * idx
    endIdx = ctypes.sizeof(ctypes.c_double) * (idx + 1)
    return struct.unpack('d', content[beginIdx:endIdx])[0]


def RowMajorIdx(cols, rowidx, colidx):
    return rowidx * cols + colidx


def PackMatrix(input, format_str):
    shape_list = [input.shape[0], input.shape[1]]
    flat_matrix = [item for sublist in input for item in sublist]
    # return shape and content
    return struct.pack(format_str.format(len(shape_list)), *shape_list), struct.pack(format_str.format(len(flat_matrix)), *flat_matrix)


if __name__ == '__main__':
    # get the directory of the current file
    dir = os.path.dirname(os.path.abspath(sys.argv[0]))
    print(dir)
    # load binary
    with open(dir + "/../build_release/src/quadrotor_control/quadJacob.dat", mode='rb') as f:
        contents = f.read()
    f.close()
    # load number of columns and rows
    Arows = int(ReadDouble(contents, 0))
    Acols = int(ReadDouble(contents, 1))
    Brows = int(ReadDouble(contents, 2))
    Bcols = int(ReadDouble(contents, 3))
    print(Arows)
    print(Acols)
    print(Brows)
    print(Bcols)
    print(len(contents))
    A = np.zeros((Arows, Acols))
    B = np.zeros((Brows, Bcols))
    AdataBegin = 4
    for i in range(Arows):
        for j in range(Acols):
            A[i, j] = ReadDouble(contents, AdataBegin + RowMajorIdx(Acols, i, j))

    BdataBegin = AdataBegin + Arows * Acols
    for i in range(Brows):
        for j in range(Bcols):
            B[i, j] = ReadDouble(contents, BdataBegin + RowMajorIdx(Bcols, i, j))
    
    print(A)
    print(B)
    # augment B matrix to make the system controllable
    # 9 - 12
    B1 = np.zeros((Brows, 1))
    B1[9] = 1.0
    Bs = np.hstack((B, B1))
    print(Bs)
    # test with lqr
    # caculate the metric
    #  eye(Nstates), eye(Ninputs + 1)
    K1, S, E = ctrl.lqr(A, Bs, np.eye(Arows), np.eye(Bcols + 1))
    print(K1)
    # save the lqr gain to a binary file

    dt = 0.05
    alpha = 0.5
    # v = 1.5
    alphagc = 0.5
    epsilon = 0.01
    # chi = 30
    v = cp.Variable(nonneg=True)
    chi = cp.Variable(nonneg=True)
    C = - 2 * v * Bs@Bs.transpose()
    X = cp.Variable((Arows, Arows), PSD=True)
    # M11 = -(X - np.eye(Arows))/dt + A@X + X@A.transpose() + C + 2 * alpha * X
    # M11 = A@X + X@A.transpose() + C + 2 * alpha * X
    M11 = (X - np.eye(Arows))/dt + A@X + X@A.transpose() + C + 2 * alpha * X
    M12 = - X
    M22 = - v/alphagc * np.eye(Arows)
    M = cp.hstack((cp.vstack((M11, M12)), cp.vstack((M12, M22))))
    Mrows, Mcols = M.shape
    constraints = [M << -epsilon*np.eye(Mrows)]

    constraints += [X << chi * np.eye(Arows)]
    constraints += [X >> np.eye(Arows)]

    # prob = cp.Problem(cp.Maximize(cp.trace(X)), constraints)
    prob = cp.Problem(cp.Minimize(1 * chi + 5 * v), constraints)
    prob.solve(solver=cp.MOSEK)

    if prob.status not in ["infeasible", "unbounded"]:
        # Otherwise, problem.value is inf or -inf, respectively.
        print("Optimal value: %s" % prob.value)
        vres = prob.variables()[0]
        chires = prob.variables()[1]
        Xres = prob.variables()[2]
        print("v: " + str(v.value))
        print("chi: " + str(chi.value))
        print("xrex: " + str(X.value))
        print("--M---")
        ccm = v.value * np.linalg.inv(X.value)
        K = Bs.T @ ccm
        print(ccm)
        print("-------K-------")
        print(K)
        # if correct, save the metric in a binary file
        sizeK, packedK = PackMatrix(K, '{}d')
        with open(dir + "/../data/ccm_result.dat", "wb") as outfile:
            outfile.write(sizeK + packedK)
    else:
        print(prob .status)
