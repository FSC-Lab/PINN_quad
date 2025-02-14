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

#ifndef QUAD_POS_CONTROLLER_H
#define QUAD_POS_CONTROLLER_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace QuadController
{
// pos controller for a rate-controlled drone
class PIDPosController
{
   public:
    static constexpr uint32_t omegaCx = 0;
    static constexpr uint32_t omegaCy = 1;
    static constexpr uint32_t omegaCz = 2;
    static constexpr uint32_t liftC = 3;
    struct Para {
        struct {
            struct {
                double pos_kp{0};
                double pos_kd{0};
                double pos_ki{0};
            } poxXYPara;
            struct {
                double pos_kp{0};
                double pos_kd{0};
                double pos_ki{0};
            } posZPara;
            double omega_kp{0};
        } ctrGains;
        struct {
            double mass;
        } quadPara;
    };
    struct Input {
        Eigen::Vector3d posRef;  // reference position
        double yawRef;           // reference yaw
        Eigen::Vector3d pos;     // current position
        Eigen::Vector3d vel;     // current velocity
        Eigen::Matrix3d RIB;     // current attitude
        double deltaT{0};        // sampling time
    };
    using ControlOutput = Eigen::Matrix<double, 4, 1>;
    const ControlOutput &GetOutput(const Input &input);
    void DisplayPara(void) const;
    void DisplayControllerStatus(void) const;
    void ResetStates(void);
    PIDPosController(const Para &para_);
    ~PIDPosController() = default;

   private:
    void CalculatePosError(void);
    void CalculateRateCommand(const Eigen::Ref<const Eigen::Vector3d> &f, const Input &input);
    Para para;
    ControlOutput output;
    Eigen::Vector3d posErr;
    Eigen::Vector3d s;          // compositie variable
    Eigen::Matrix3d kp;         // pos gain
    Eigen::Matrix3d kd;         // vel gain
    Eigen::Matrix3d ki;         // integral gain
    Eigen::Vector3d fp;         // virtual lift from sliding variable
    Eigen::Vector3d f;          // total lift
    Eigen::Vector3d mg;         // gravity force
    Eigen::Matrix3d RIBdc;      // current desired attitude;
    Eigen::Matrix3d RIBdp;      // previous desired attitude
    Eigen::Vector3d omegadEst;  // estimation of the
    Eigen::Vector3d eR;         // previous attitude error
    Eigen::Vector3d omegac;     // rate command
};
// circle following for a rate controlled drone
};  // namespace QuadController

#endif