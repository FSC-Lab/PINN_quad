/*************************************************************

rigid body simulation test

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

*************************************************************/

#include "quad_pos_controller.h"
#include <iostream>
#include "kinematics_utils.h"
#include "utility_functions.h"

namespace QuadController
{
PIDPosController::PIDPosController(const Para &para_) : para(para_)
{
    // load control parameters
    kp.setZero();
    kd.setZero();
    ki.setZero();

    kp(mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_X) = para_.ctrGains.poxXYPara.pos_kp;
    kp(mathauxiliary::VECTOR_Y, mathauxiliary::VECTOR_Y) = para_.ctrGains.poxXYPara.pos_kp;
    kp(mathauxiliary::VECTOR_Z, mathauxiliary::VECTOR_Z) = para_.ctrGains.posZPara.pos_kp;

    kd(mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_X) = para_.ctrGains.poxXYPara.pos_kd;
    kd(mathauxiliary::VECTOR_Y, mathauxiliary::VECTOR_Y) = para_.ctrGains.poxXYPara.pos_kd;
    kd(mathauxiliary::VECTOR_Z, mathauxiliary::VECTOR_Z) = para_.ctrGains.posZPara.pos_kd;
    // initialized internal variables
    RIBdc.setIdentity();
    RIBdp.setIdentity();

    omegadEst.setZero();

    mg.setZero();
    mg(mathauxiliary::VECTOR_Z) = -para.quadPara.mass * 9.81;
}

const PIDPosController::ControlOutput &PIDPosController::GetOutput(const Input &input)
{
    // calculate position error
    posErr = input.pos - input.posRef;
    // calculate sliding variable
    s = input.vel + kp * posErr / sqrt(1 + posErr.transpose() * posErr);
    // sliding variable to desired lift
    fp = -kd * s;
    f = fp - mg;
    CalculateRateCommand(f, input);
    // assemble output
    output.segment(0, 3) = omegac;
    output(liftC) = f.norm();
    return output;
}

void PIDPosController::CalculateRateCommand(const Eigen::Ref<const Eigen::Vector3d> &f, const Input &input)
{
    // convert desired atttiude to rate command
    // store the previous desired attitude
    RIBdp = RIBdc;
    RIBdc = KinematicsUtils::GetDesiredAttitude(f, input.yawRef);
    // estimatet the desired angular velocity
    // omegadEst = KinematicsUtils::GetAttitudeError(RIBdc, RIBdp);
    // the attitude error:
    eR = KinematicsUtils::GetAttitudeError(RIBdc, input.RIB);

    // desired attitude is the combination of the omega estimation and the atttitude error
    omegac = -para.ctrGains.omega_kp * eR;
}

void PIDPosController::DisplayPara(void) const
{
    std::cout << "---------PID controller parameter-----------\n";
    std::cout << "------------ position channel -------------\n";
    std::cout << "kp:\n";
    std::cout << kp << '\n';
    std::cout << "kd:\n";
    std::cout << kd << '\n';
    std::cout << "ki:\n";
    std::cout << ki << '\n';
    std::cout << "mg: \n";
    std::cout << mg << '\n';
    std::cout << "------------ attitude channel -------------\n";
    std::cout << "omega kp:\n";
    std::cout << para.ctrGains.omega_kp << '\n';
}

void PIDPosController::DisplayControllerStatus(void) const
{
    std::cout << "---------PID controller states-----------\n";
    std::cout << "Pos error: [" << posErr(mathauxiliary::VECTOR_X) << "] , [" << posErr(mathauxiliary::VECTOR_Y)
              << "], [" << posErr(mathauxiliary::VECTOR_Z) << "]. \n";
    std::cout << "composite variable: s: [" << s(mathauxiliary::VECTOR_X) << "], [" << s(mathauxiliary::VECTOR_Y)
              << "], [" << s(mathauxiliary::VECTOR_Z) << "]. \n";
    std::cout << "virtual lift p: [" << fp(mathauxiliary::VECTOR_X) << "], [" << fp(mathauxiliary::VECTOR_Y) << "], ["
              << fp(mathauxiliary::VECTOR_Z) << "]. \n";
    std::cout << "total lift: [" << f(mathauxiliary::VECTOR_X) << "], [" << f(mathauxiliary::VECTOR_Y) << "], ["
              << f(mathauxiliary::VECTOR_Z) << "]. \n";
    std::cout << "eR: [" << eR(mathauxiliary::VECTOR_X) << "], [" << eR(mathauxiliary::VECTOR_Y) << "], ["
              << eR(mathauxiliary::VECTOR_Z) << "]. \n";
    std::cout << "omegaEst: [" << omegadEst(mathauxiliary::VECTOR_X) << "], [" << omegadEst(mathauxiliary::VECTOR_Y)
              << "], [" << omegadEst(mathauxiliary::VECTOR_Z) << "]. \n";
    std::cout << "omegac: [" << omegac(mathauxiliary::VECTOR_X) << "], [" << omegac(mathauxiliary::VECTOR_Y) << "], ["
              << omegac(mathauxiliary::VECTOR_Z) << "]. \n";
}
void PIDPosController::ResetStates(void) {}

}  // namespace QuadController