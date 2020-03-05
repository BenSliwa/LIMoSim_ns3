#include "quadrotor.h"

#include <math.h>
#include <cmath>

namespace LIMoSim {
namespace Energy {
namespace Models {

Quadrotor::Quadrotor():
    m_distanceToRotorShaft_m (0.175),
    m_dragCoeff_Nm_p_rad_p_s(2.2518e-8),
    m_mass_kg(1.3),
    m_rotorInertia_kgm2(4.190e-5),
    m_thrustFactor_N_p_rad_p_s(3.8305e-6),
    m_vehicleInertia_kg_m2 (0.081, 0.081, 0.142),
    m_viscousDampingCoeff_N_ms_p_rad(0.2e-3),
    m_previousMotorSpeeds {0,0,0,0},
    m_motorSpeeds {0,0,0,0},
    m_motorTorques {0,0,0,0},
    m_efficiencyFactors {0,0,0,0},
    T(0), u1(0), u2(0), u3(0)
{

}

double Quadrotor::step(
        double _timeDelta_s,
        Vector3d _linearAcc,
        Orientation3d _angularAcc
        )
{
    deriveControlInputs( _linearAcc, _angularAcc);
    computeMotorSpeeds();
    computeMotorTorques();
    computeEfficiencyFactors();
    return computeEnergyConsumption(_timeDelta_s);
}

void Quadrotor::deriveControlInputs(Vector3d _linearAcc, Orientation3d _angularAcc)
{
    T = m_mass_kg * (_linearAcc - Vector3d(0,0,-g)).norm();
    u1 = m_vehicleInertia_kg_m2.x * _angularAcc.roll() / m_distanceToRotorShaft_m;
    u2 = m_vehicleInertia_kg_m2.y * _angularAcc.pitch()/ m_distanceToRotorShaft_m;
    u3 = m_vehicleInertia_kg_m2.z * _angularAcc.yaw();
}

void Quadrotor::computeMotorSpeeds()
{
    double A = (u2 + u1) / m_thrustFactor_N_p_rad_p_s ;
    double B = (u2 - u1) / m_thrustFactor_N_p_rad_p_s ;
    double L = 0.5 * (T - m_thrustFactor_N_p_rad_p_s * A) / m_thrustFactor_N_p_rad_p_s;
    double R = 0.5 * (u3 - m_dragCoeff_Nm_p_rad_p_s * B) / m_dragCoeff_Nm_p_rad_p_s;
    m_motorSpeeds[0] = sqrt(std::abs(0.5 * (L + R)));
    m_motorSpeeds[1] = sqrt(std::abs(0.5 * (L - R)));
    m_motorSpeeds[2] = sqrt(std::abs(m_motorSpeeds[0] * m_motorSpeeds[0] + u2/m_thrustFactor_N_p_rad_p_s));
    m_motorSpeeds[3] = sqrt(std::abs(m_motorSpeeds[1] * m_motorSpeeds[1] - u1/m_thrustFactor_N_p_rad_p_s));
}

void Quadrotor::computeMotorTorques()
{
    for(unsigned int i = 0; i < 4; i++) {
        m_motorTorques[i] = m_rotorInertia_kgm2 * (m_motorSpeeds[i] - m_previousMotorSpeeds[i]) +
                (m_dragCoeff_Nm_p_rad_p_s * m_motorSpeeds[i] * m_motorSpeeds[i]) +
                (m_viscousDampingCoeff_N_ms_p_rad * m_motorSpeeds[i]);
    }

    // prepare previous motor speeds for next iteration step
    for (unsigned int i=0; i < 4; i++) {
        m_previousMotorSpeeds[i] = m_motorSpeeds[i];
    }
}

void Quadrotor::computeEfficiencyFactors()
{
    double A, B, C, D, ec = s_efficiencyCompensation;
    for (unsigned int i = 0; i < 4; i++) {
        A = 0, B = 0,  C = 0, D = 0;
        A = a1 * m_motorSpeeds[i] * m_motorSpeeds[i] + b1 * m_motorSpeeds[i] + c1;
        B = a2 * m_motorSpeeds[i] * m_motorSpeeds[i] + b2 * m_motorSpeeds[i] + c2;
        C = a3 * m_motorSpeeds[i] * m_motorSpeeds[i] + b3 * m_motorSpeeds[i] + c3;
        D = a4 * m_motorSpeeds[i] * m_motorSpeeds[i] + b4 * m_motorSpeeds[i] + c4;
        m_efficiencyFactors[i] = std::max(ec, std::abs(A * m_motorTorques[i] * m_motorTorques[i] * m_motorTorques[i] +
                B * m_motorTorques[i] * m_motorTorques[i]+ C * m_motorTorques[i] + D));
    }
}

double Quadrotor::computeEnergyConsumption(double _timeDelta_s)
{
    double ec = 0;
    for(unsigned int i = 0; i < 4; i++) {
        ec += m_motorTorques[i] * m_motorSpeeds[i] * _timeDelta_s / m_efficiencyFactors[i];
    }
    return ec;
}


} // namespace Models
} // namespace Energy
} // namespace LIMoSim
