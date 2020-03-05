#ifndef UAV_LOCOMOTION_H
#define UAV_LOCOMOTION_H


#include "LIMoSim/mobility/uav/reynolds/locomotion.h"

#include "LIMoSim/energy/models/quadrotor/quadrotor.h"

namespace LIMoSim {

class UAV;
typedef std::pair<Vector3d,Vector3d> VectorPair;

using namespace Energy::Models;

class UAV_Locomotion : public Locomotion
{
public:
    UAV_Locomotion(std::string _agentId);

    static std::string name();

    UAV* getAgent();

    Vector3d getBrakeAccelerationMax();
    Vector3d getAccelerationComponentsMax();
    Vector3d getVelocityComponentsMax();

    void setBrakeAccelerationMax(Vector3d _brakeAccelerationMax_mps2);
    void setAccelerationComponentsCoeffMax(Vector3d _accelerationComponentsCoeffMax);
    void setVelocityComponentsCoeffMax(Vector3d _velocityComponentsCoeffMax);

    // Locomotion interface
public:
    virtual Vector3d constrainAcceleration(Vector3d _acc) override;
    virtual Vector3d constrainVelocity(Vector3d _velocity) override;
    virtual Steering normalizeSteering(const Steering &_steering) override;
    LocomotionUpdate apply(Steering steering, double _timeDelta_s);

protected:
    Vector3d m_brake_acceleration_max_mps2;
    Vector3d m_acceleration_components_coeff_max;
    Vector3d m_velocity_components_coeff_max;

    VectorPair applyOrientationSteering(Orientation3d _orientationSteering, double _timeDelta_s);
    std::vector<Vector3d> applyPositionSteering(Vector3d _positionSteering, double _timeDelta_s);

    Vector3d computeInducedBraking(Vector3d _acc);
    Vector3d constrainInducedBraking(Vector3d _acc);
    Vector3d projectInAgentSpace(Vector3d _vec);
    Vector3d fromAgentSpace(Vector3d _vec);
    Vector3d getDirection();

private:
    Quadrotor m_quadrotorEnergyModel;
};

} // namespace LIMoSim

#endif // UAV_LOCOMOTION_H
