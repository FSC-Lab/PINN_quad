/*************************************************************

rigid body simulation test

Copyright (C) 2023  Longhao Qian

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

#include <iostream>
#include "Model.h"
#include "quadrotor_model.h"
#include "quad_pos_controller.h"

int main(void)
{
    // read the metric
    FileIO::ReadBinary<double> refData;
    const char fileName[]{"../../../data/ccm_result.dat"};
    if (!refData.Open(fileName)) {
        std::cout<<"can not open file\n";
        return 1;
    }
    // get the ccm
    refData.ClearAndRead();
    auto &refDataArr = refData.GetDataBuffer(); // add a flag
    // to avoid segmentation fault
    uint32_t numofRows = static_cast<uint32_t>(refDataArr[0]);
    uint32_t numofCols = static_cast<uint32_t>(refDataArr[1]);
    std::cout<<numofRows<<", "<< numofCols<<"\n";
    Eigen::MatrixXd ccmK(numofRows, numofCols);
    ccmK.setZero();
    uint32_t startIdx = 2;
    // for (auto t : refDataArr) {
    //     std::cout<<t<<'\n';
    // }
    for (uint32_t i = 0; i < numofRows;i++) {
        for(uint32_t j = 0; j < numofCols; j++){
            ccmK(i, j) = refDataArr[mathauxiliary::MatrixElementRowMajor(startIdx, numofCols, i, j)];
        }
    }
    std::cout<<ccmK<<'\n';
    // define solver
    Mdss::SolverConfig config1;
    config1.eposilon = 0.00001;
    config1.adaptive_step = false;
    config1.frame_step = 0.01;
    config1.mim_step = 0.005;
    config1.start_time = 0.0;
    config1.solver_type = RungeKuttaFamily::DORMANDPRINCE;
    config1.loggingconfig.filename = "ccm_test_1";
    config1.loggingconfig.uselogging = true;
    config1.loglevel = Mdss::LOGLEVEL_ERROR;
    Mdss::Model model(config1);
    // parameter
    Quadrotor::RateControlledQuad::Para para1;
    // para1.m = 1.0;
    para1.m = 2.3;
    para1.ta[0] = 10;
    para1.ta[1] = 10;
    para1.ta[2] = 10;
    para1.tl = 0.1;
    // define quadrotor
    Quadrotor::RateControlledQuad::StateQuaternion icQuad;
    Eigen::Vector3d euler0;
    euler0.setZero();
    euler0(0) = 0;
    euler0(1) = 0;
    euler0(2) = 0;
    icQuad.Quaternion = mathauxiliary::GetQuaterionFromRulerAngle(euler0);
    // icQuad.Quaternion.setZero();
    icQuad.XI0.setZero();
    icQuad.VI0.setZero();
    icQuad.Omega0.setZero();
    icQuad.L = 10.0;

    Quadrotor::RateControlledQuad quadrotor(model, para1, icQuad);
    auto &acc = quadrotor.GetInertialAcc();
    auto &euler = quadrotor.GetEuler();
    auto &omegaDot = quadrotor.GetOmegaDot();
    auto &Rib = quadrotor.GetRib();
    auto &xI = quadrotor.GetxI();
    auto &vI = quadrotor.GetvI();
    model.DefineDataLogging(acc, "acc");
    model.DefineDataLogging(omegaDot, "omegaDot");
    model.DefineDataLogging(xI, "xI");
    model.DefineDataLogging(vI, "vI");
    model.DefineDataLogging(euler, "euler");

    // simple PID from external inputs
    model.Compile();
    VectorXd extern_input;
    model.ReshapeExternalInputVector(extern_input);
    extern_input.setZero();

    constexpr uint32_t numOfSteps = 1000;

    QuadController::PIDPosController::Para para;
    para.ctrGains.omega_kp = 20;
    para.ctrGains.posZPara.pos_kd = 5;
    para.ctrGains.posZPara.pos_kp = 5;
    para.ctrGains.poxXYPara.pos_kd = 5;
    para.ctrGains.poxXYPara.pos_kp = 2;
    para.quadPara.mass = para1.m;

    QuadController::PIDPosController controller(para);
    QuadController::PIDPosController::Input inputT;
    controller.DisplayPara();
    inputT.deltaT = config1.frame_step;

    // reference postion
    inputT.posRef(mathauxiliary::VECTOR_X) = -5;
    inputT.posRef(mathauxiliary::VECTOR_Y) = 5;
    inputT.posRef(mathauxiliary::VECTOR_Z) = 5;

    inputT.yawRef = 0;

    // get the external input mapping
    auto omegaCxIdx = model.GetExternalInputIndex(quadrotor.GetOmegaComIdx(), 0);
    auto omegaCxIdy = model.GetExternalInputIndex(quadrotor.GetOmegaComIdx(), 1);
    auto omegaCxIdz = model.GetExternalInputIndex(quadrotor.GetOmegaComIdx(), 2);

    auto liftCId = model.GetExternalInputIndex(quadrotor.GetLiftLagIdx(), 0);
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    double posx = 0;
    double posy = 0;
    double posz = 0;

    roll = model.GetSubsystemOutput(euler.GetIdx(0, 0));
    pitch = model.GetSubsystemOutput(euler.GetIdx(1, 0));
    yaw = model.GetSubsystemOutput(euler.GetIdx(2, 0));
    std::cout << "omegaCidx: " << quadrotor.GetOmegaComIdx() << '\n';
    std::cout << model.subsystem_list[quadrotor.GetOmegaComIdx()]->GetSystemInfo().input_connection << '\n';
    std::cout << "roll :[" << roll << "], pitch [" << pitch << "], yaw [" << yaw << "]. \n";

    for (uint32_t i = 0; i < numOfSteps; i++) {
        // get output
        for (uint32_t i = 0; i < 3; i++) {
            inputT.pos(mathauxiliary::VECTOR_X +
                       i) = model.GetSubsystemOutput(xI.GetIdx(mathauxiliary::VECTOR_X + i, 0));
            inputT.vel(mathauxiliary::VECTOR_X +
                       i) = model.GetSubsystemOutput(vI.GetIdx(mathauxiliary::VECTOR_X + i, 0));
        }
        for (uint32_t i = 0; i < 3; i++) {
            for (uint32_t j = 0; j < 3; j++) {
                inputT.RIB(i, j) = model.GetSubsystemOutput(Rib.GetIdx(i, j));
            }
        }
        // calculate control law
        auto &output = controller.GetOutput(inputT);
        extern_input(omegaCxIdx) = output(controller.omegaCx);
        extern_input(omegaCxIdy) = output(controller.omegaCy);
        extern_input(omegaCxIdz) = output(controller.omegaCz);
        extern_input(liftCId) = output(controller.liftC);
        // get euler angles
        roll = model.GetSubsystemOutput(euler.GetIdx(0, 0));
        pitch = model.GetSubsystemOutput(euler.GetIdx(1, 0));
        yaw = model.GetSubsystemOutput(euler.GetIdx(2, 0));
        // omegaDotx = model.GetSubsystemOutput(omegaDot.GetIdx(0, 0));
        // omegaDoty = model.GetSubsystemOutput(omegaDot.GetIdx(1, 0));
        // omegaDotz = model.GetSubsystemOutput(omegaDot.GetIdx(2, 0));
        // omegaErrx = model.GetSubsystemOutput(quadrotor.GetOmegaComIdx(), 0);
        // omegaErry = model.GetSubsystemOutput(quadrotor.GetOmegaComIdx(), 1);
        // omegaErrz = model.GetSubsystemOutput(quadrotor.GetOmegaComIdx(), 2);
        // model.GetExternalInputs(extern_input);
        // put input
        model.Run_Update(controller.GetOutput(inputT));
    }

    posx = model.GetSubsystemOutput(xI.GetIdx(0, 0));
    posy = model.GetSubsystemOutput(xI.GetIdx(1, 0));
    posz = model.GetSubsystemOutput(xI.GetIdx(2, 0));
    std::cout << "posx :[" << posx << "], posy [" << posy << "], posz [" << posz << "]. \n";
    controller.DisplayControllerStatus();
    model.PostRunProcess();
    return 0;
}