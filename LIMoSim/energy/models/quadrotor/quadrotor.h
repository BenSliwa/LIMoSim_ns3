#ifndef QUADROTOR_H
#define QUADROTOR_H

#include <vector>

#include "LIMoSim/world/vector3d.h"
#include "LIMoSim/world/orientation3d.h"

namespace LIMoSim {
namespace Energy {
namespace Models {

class Quadrotor
{
public:
    Quadrotor();

    double step(double _timeDelta_s,
            Vector3d _linearAcc,
            Orientation3d _angularAcc);

private:
    // Model parameters
    double m_distanceToRotorShaft_m;
    double m_dragCoeff_Nm_p_rad_p_s;
    double m_mass_kg;
    double m_rotorInertia_kgm2;
    double m_thrustFactor_N_p_rad_p_s;
    Vector3d m_vehicleInertia_kg_m2;
    double m_viscousDampingCoeff_N_ms_p_rad;

    // Model memory for differentiation
    double m_previousMotorSpeeds[4];

    // Model indermediate values
    double m_motorSpeeds[4];
    double m_motorTorques[4];
    double m_efficiencyFactors[4];

    // Model control inputs
    double T, u1, u2, u3;

    // constants
    static constexpr double g = 9.81;

    // efficiency factor model parameters
    static constexpr double a1 = -1.72e-5;
    static constexpr double a2 = 1.95e-5;
    static constexpr double a3 = -6.98e-6;
    static constexpr double a4 = 4.09e-7;
    static constexpr double b1 =  0.014;
    static constexpr double b2 = -0.0157;
    static constexpr double b3 = 5.656e-3;
    static constexpr double b4 = -3.908e-4;
    static constexpr double c1 = -0.8796;
    static constexpr double c2 = 0.3385;
    static constexpr double c3 = 0.2890;
    static constexpr double c4 = 0.1626;
    static constexpr double s_efficiencyCompensation = 0.76;


    void deriveControlInputs(Vector3d _linearAcc,
            Orientation3d _angularAcc);
    void computeMotorSpeeds();
    void computeMotorTorques();
    void computeEfficiencyFactors();
    double computeEnergyConsumption(double _timeDelta_s);
    void cleanup();
};

} // namespace Models
} // namespace Energy
} // namespace LIMoSim

#endif // QUADROTOR_H
