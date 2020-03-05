#include "locomotion.h"
#include "LIMoSim/mobility/uav/reynolds/reynoldsmodel.h"
#include "LIMoSim/mobility/uav/uav.h"
#include "LIMoSim/world/vehiclemanager.h"

namespace LIMoSim {

Locomotion::Locomotion(std::string _agentId, std::string _name):
    m_agentId(_agentId),
    m_name(_name)
{}

Locomotion::~Locomotion()
{

}

Vector3d Locomotion::constrainAcceleration(Vector3d _acc)
{
    return (_acc.norm() && (_acc.norm() > m_acceleration_max_mps2)) ?
                _acc.normed() * m_acceleration_max_mps2 :
                _acc;
}

Vector3d Locomotion::constrainVelocity(Vector3d _velocity)
{
    return (_velocity.norm() && (_velocity.norm() > m_velocity_max_mps)) ?
                _velocity.normed() * m_velocity_max_mps:
                _velocity;
}

Vector3d Locomotion::maximizeVelocity(Vector3d _velocity)
{
    if (_velocity.norm()) {
        return _velocity.normed() * m_velocity_max_mps;
    }
    return _velocity;
}

Vector3d Locomotion::modulateVelocity(Vector3d _velocity, double _speed)
{
    if (_velocity.norm()) {
        return _velocity.normed() * _speed;
    }
    return _velocity;
}

Vehicle *Locomotion::getAgent()
{
    return VehicleManager::getInstance()->getVehicle(m_agentId);
}

std::string Locomotion::getName()
{
    return m_name;
}

double Locomotion::getAccelerationMax()
{
    return m_acceleration_max_mps2;
}

double Locomotion::getVelocityMax()
{
    return m_velocity_max_mps;
}

Orientation3d Locomotion::getOrientationAccelerationMax()
{
    return m_orientation_acceleration_max_radps2;
}

Orientation3d Locomotion::getOrientationVelocityMax()
{
    return m_orientation_velocity_max_radps;
}

void Locomotion::setAccelerationMax(double _accMax)
{
    m_acceleration_max_mps2 = _accMax;
}

void Locomotion::setVelocityMax(double _velMax)
{
    m_velocity_max_mps = _velMax;
}

void Locomotion::setOrientationAccelerationMax(Orientation3d _orientationAccelerationMax)
{
    m_orientation_acceleration_max_radps2 = _orientationAccelerationMax;
}

void Locomotion::setOrientationVelocityMax(Orientation3d _orientationVelocityMax)
{
    m_orientation_velocity_max_radps = _orientationVelocityMax;
}

Steering Locomotion::normalizeSteering(const Steering &_steering)
{
    Steering normalizedSteering = _steering;

    // normalize orientation steering

    // normalize position steering
    //if (_steering.position.norm() > getAccelerationMax()) {
        normalizedSteering.position = _steering.position.norm() ? _steering.position.normed() * getAccelerationMax() : Vector3d();
    //}
        normalizedSteering.orientation.setPitch(std::min(normalizedSteering.orientation.pitch(), getOrientationAccelerationMax().pitch()));
        normalizedSteering.orientation.setRoll(std::min(normalizedSteering.orientation.roll(), getOrientationAccelerationMax().roll()));
        normalizedSteering.orientation.setYaw(std::min(normalizedSteering.orientation.yaw(), getOrientationAccelerationMax().yaw()));

    return normalizedSteering;
}

}
