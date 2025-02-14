/*************************************************************

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
#include <Eigen/Dense>
#include <string_view>
#include "quadrotor_model.h"
#include "io_proc.h"
#include "utility_functions.h"
#include "quad_pos_controller.h"

// there is a bug here
// putting this function before creating model cause an compuational error
Eigen::MatrixXd GetMatrix(const char* filename) {
    Eigen::MatrixXd res;
    // read the metric
    FileIO::ReadBinary<double> refData;

    if (!refData.Open(filename)) {
        std::cout<<"can not open file\n";
        return res;
    }
    // get the ccm
    refData.ClearAndRead();
    auto &refDataArr = refData.GetDataBuffer(); // add a flag
    // to avoid segmentation fault
    uint32_t numofRows = static_cast<uint32_t>(refDataArr[0]);
    uint32_t numofCols = static_cast<uint32_t>(refDataArr[1]);
    std::cout<<numofRows<<", "<< numofCols<<"\n";
    res.resize(numofRows, numofCols);
    res.setZero();
    constexpr uint32_t startIdx = 2;
    for (uint32_t i = 0; i < numofRows;i++) {
        for(uint32_t j = 0; j < numofCols; j++){
            res(i, j) = refDataArr[mathauxiliary::MatrixElementRowMajor(startIdx, numofCols, i, j)];
        }
    }
    std::cout<<res<<'\n';
    return res;

}

int main(int argc, char **argv)
{

    //  const char fileName[]{"../../../data/ccm_result.dat"};
    Mdss::SolverConfig config1;
    config1.eposilon = 0.00001;
    config1.adaptive_step = false;
    config1.frame_step = 0.01;
    config1.mim_step = 0.005;
    config1.start_time = 0.0;
    config1.solver_type = RungeKuttaFamily::DORMANDPRINCE;
    config1.loggingconfig.filename = "ccm_result";
    config1.loggingconfig.uselogging = true;
    config1.loglevel = Mdss::LOGLEVEL_ERROR;
    Mdss::Model model(config1);
    // parameter
    Quadrotor::RateControlledQuad::Para para1;
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
    icQuad.L = para1.m * 9.81;

    Quadrotor::RateControlledQuad quadrotor(model, para1, icQuad);
    auto &acc = quadrotor.GetInertialAcc();
    auto &euler = quadrotor.GetEuler();
    auto &omegaDot = quadrotor.GetOmegaDot();
    auto &Rib = quadrotor.GetRib();
    auto &xI = quadrotor.GetxI();
    auto &vI = quadrotor.GetvI();
    auto &omega = quadrotor.GetOmega();
    auto &quaternion = quadrotor.GetQuaternion();
    auto &ftotal = quadrotor.GetTotoalForce();
    model.DefineDataLogging(acc, "acc");
    model.DefineDataLogging(omegaDot, "omegaDot");
    model.DefineDataLogging(xI, "xI");
    model.DefineDataLogging(vI, "vI");
    model.DefineDataLogging(euler, "euler");

    // simple PID from external inputs
    // compile the model
    if (!model.Compile()) {
        std::cout << "ERROR: Model compile failed, check connections and subsystem definitions!\n";
        return 1;
    }
    Eigen::VectorXd extern_input;
    model.ReshapeExternalInputVector(extern_input);
    auto liftcomIdx = model.GetExternalInputIndex(quadrotor.GetLiftLagIdx(), 0);
    // get the external input mapping
    auto omegaCxIdx = model.GetExternalInputIndex(quadrotor.GetOmegaComIdx(), 0);
    auto omegaCxIdy = model.GetExternalInputIndex(quadrotor.GetOmegaComIdx(), 1);
    auto omegaCxIdz = model.GetExternalInputIndex(quadrotor.GetOmegaComIdx(), 2);


    // states
    constexpr uint32_t vSize = 3;
    constexpr uint32_t omegaSize = 3;
    constexpr uint32_t xSize = 3;
    constexpr uint32_t qSize = 4;
    constexpr uint32_t fSize = 1;
    constexpr uint32_t numOfStates = vSize + omegaSize + xSize + qSize + fSize;
    constexpr uint32_t vBegin = 0;
    constexpr uint32_t omegaBegin = vSize;
    constexpr uint32_t xBegin = omegaBegin + omegaSize;
    constexpr uint32_t qBegin = xBegin + xSize;
    constexpr uint32_t fBegin = qBegin + qSize;
    /*
    KINEMATICS_STATE_VIx = 0,
    KINEMATICS_STATE_VIy,
    KINEMATICS_STATE_VIz,
    KINEMATICS_STATE_OmegaBIx,
    KINEMATICS_STATE_OmegaBIy,
    KINEMATICS_STATE_OmegaBIz,
    KINEMATICS_STATE_XIx,
    KINEMATICS_STATE_XIy,
    KINEMATICS_STATE_XIz,
    KINEMATICS_STATE_q0,
    KINEMATICS_STATE_q1,
    KINEMATICS_STATE_q2,
    KINEMATICS_STATE_q3,
    */

    // get the equilbrium input
    Eigen::Vector4d u0;
    u0.setZero();
    u0(liftcomIdx) = icQuad.L;
    // state variable
    Eigen::Matrix<double, numOfStates, 1> x;
    x.setZero();
    // target state
    Eigen::Matrix<double, numOfStates, 1> xt;
    xt.setZero();
    xt(xBegin) = 0.3;
    xt(qBegin) = 1.0;

    // temporary control input
    Eigen::Matrix<double, 5, 1> deltau;
    deltau.setZero();

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
    Eigen::MatrixXd ccmK = GetMatrix("../../../data/ccm_result.dat");
    // run the simulation
    uint32_t numOfSteps = 3000;
    // need an estimation of the real lift
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
        // get output and stored in the state
        for (uint32_t i = 0; i < 3; i++) {
            x(omegaBegin + i) = model.GetSubsystemOutput(omega.GetIdx(mathauxiliary::VECTOR_X + i, 0));
            x(xBegin + i) = model.GetSubsystemOutput(xI.GetIdx(mathauxiliary::VECTOR_X + i, 0));
            x(vBegin +i) = model.GetSubsystemOutput(vI.GetIdx(mathauxiliary::VECTOR_X + i, 0));
        }
        for (uint32_t i = 0; i < 4; i++) {
            x(qBegin +i) = model.GetSubsystemOutput(quaternion.GetIdx(mathauxiliary::VECTOR_X + i, 0));
        }

        // calculate control law
        deltau = ccmK * (xt - x);
        extern_input = u0 + deltau.head(4);
        model.Run_Update(extern_input);
        // model.Run_Update(controller.GetOutput(inputT));
    }
    double posx = model.GetSubsystemOutput(xI.GetIdx(0, 0));
    double posy = model.GetSubsystemOutput(xI.GetIdx(1, 0));
    double posz = model.GetSubsystemOutput(xI.GetIdx(2, 0));
    std::cout << "posx :[" << posx << "], posy [" << posy << "], posz [" << posz << "]. \n";
    model.PostRunProcess();
    return 0;
}