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

#include "psd_basis_gen.h"
#include "fusion.h"
#include "fusion_p.h"
#include "simple_lmi.h"
#include "quadrotor_model.h"
#include "jacobian_manager.h"
#include "io_proc.h"

using namespace mosek::fusion;
using namespace monty;

int main(int argc, char **argv)
{
    // define solver
    Mdss::SolverConfig config1;
    config1.eposilon = 0.00001;
    config1.adaptive_step = false;
    config1.frame_step = 0.01;
    config1.mim_step = 0.005;
    config1.start_time = 0.0;
    config1.solver_type = RungeKuttaFamily::DORMANDPRINCE;
    config1.loggingconfig.filename = "quadrotor_pos_control_1";
    config1.loggingconfig.uselogging = false;
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
    model.DefineDataLogging(acc, "acc");
    model.DefineDataLogging(omegaDot, "omegaDot");
    model.DefineDataLogging(xI, "xI");
    model.DefineDataLogging(vI, "vI");
    model.DefineDataLogging(euler, "euler");

    // simple PID from external inputs
    model.Compile();
    Eigen::VectorXd extern_input;
    model.ReshapeExternalInputVector(extern_input);
    auto liftcomIdx = model.GetExternalInputIndex(quadrotor.GetLiftLagIdx(), 0);
    extern_input(liftcomIdx) = icQuad.L;

    Mdss::AutoJacobian::AutoJacobianMgr jacob(model);
    model.GetExternalInputs(extern_input);
    model.UpdateAfterOverride();

    constexpr uint32_t vSize = 3;
    constexpr uint32_t omegaSize = 3;
    constexpr uint32_t xSize = 3;
    constexpr uint32_t qSize = 4;

    if (!jacob.SelectOutput(xI)) {
        std::cout << "select output xI failed!\n";
        return 1;
    }
    if (!jacob.SelectOutput(vI)) {
        std::cout << "select output vI failed!\n";
        return 1;
    }

    if (!jacob.SelectOutput(euler)) {
        std::cout << "select output euler failed!\n";
        return 1;
    }
    jacob.Compile();

    const auto &resJacob = jacob.GetJacobian();
    std::cout << "------\n";
    jacob.CalculateJacobian();
    std::cout << "Matrix A is: \n";
    std::cout << resJacob.A << '\n';
    std::cout << "Matrix B is: \n";
    std::cout << resJacob.B << '\n';

    uint32_t rows = resJacob.A.cols();
    uint32_t cols = resJacob.A.rows() * resJacob.B.cols();

    FileIO::WriteBinary<double, 128> dataLogger;
    // save A B matrices to a binary file
    // rows of A, cols of A
    // rows of B, cols of B
    // A data
    // B data
    dataLogger.Open("quadJacob.dat", std::ios::trunc);
    dataLogger.WriteToBuffer(resJacob.A.rows());
    dataLogger.WriteToBuffer(resJacob.A.cols());
    dataLogger.WriteToBuffer(resJacob.B.rows());
    dataLogger.WriteToBuffer(resJacob.B.cols());

    for (uint32_t i = 0; i < resJacob.A.rows(); i++) {
        for (uint32_t j = 0; j < resJacob.A.cols(); j++) {
            dataLogger.WriteToBuffer(resJacob.A(i, j));
        }
    }

    for (uint32_t i = 0; i < resJacob.B.rows(); i++) {
        for (uint32_t j = 0; j < resJacob.B.cols(); j++) {
            dataLogger.WriteToBuffer(resJacob.B(i, j));
        }
    }

    dataLogger.FlushBuffer();
    dataLogger.Close();

    // Eigen::MatrixXd C;
    // C.resize(rows, cols);
    // C.setZero();
    // C.middleCols(0, resJacob.B.cols()) = resJacob.B;
    // for (uint32_t i = 1; i < resJacob.A.cols(); i++) {
    //     C.middleCols(i * resJacob.B.cols(), resJacob.B.cols()) = resJacob.A * C.middleCols((i - 1) *
    //     resJacob.B.cols(),
    //                                                                                        resJacob.B.cols());
    // }
    // auto D = C * C.transpose();
    // std::cout << D << '\n';

    // calculate the contraction metric at the equilibrium

    return 0;
}