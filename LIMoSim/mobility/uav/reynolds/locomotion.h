#ifndef LOCOMOTION_H
#define LOCOMOTION_H

#include <string>
#include "LIMoSim/mobility/uav/reynolds/steering.h"
#include "LIMoSim/mobility/uav/reynolds/locomotionupdate.h"
#include "LIMoSim/world/orientation3d.h"

namespace LIMoSim {

class Vehicle;
class ReynoldsModel;

class Locomotion
{
public:
    Locomotion(std::string _agentId, std::string _name);
    virtual ~Locomotion();


    virtual Vector3d constrainAcceleration(Vector3d _acc);
    virtual Vector3d constrainVelocity(Vector3d _velocity);
    virtual Vector3d maximizeVelocity(Vector3d _velocity);
    virtual Vector3d modulateVelocity(Vector3d _velocity, double _speed);
    Vehicle *getAgent();
    std::string getName();

    double getAccelerationMax();
    double getVelocityMax();
    Orientation3d getOrientationAccelerationMax();
    Orientation3d getOrientationVelocityMax();

    void setAccelerationMax(double _accMax);
    void setVelocityMax(double _velMax);
    void setOrientationAccelerationMax(Orientation3d _orientationAccelerationMax);
    void setOrientationVelocityMax(Orientation3d _orientationVelocityMax);

    virtual Steering normalizeSteering(const Steering &_steering);

    virtual LocomotionUpdate apply(Steering steering, double _timeDelta_s) = 0;

protected:
    double m_acceleration_max_mps2 = 35;
    double m_velocity_max_mps = 17;
    Orientation3d m_orientation_acceleration_max_radps2;
    Orientation3d m_orientation_velocity_max_radps;

private:
    std::string m_agentId;
    std::string m_name;

};

}

#endif // LOCOMOTION_H
