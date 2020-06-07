#include "uav_locomotion.h"
#include "LIMoSim/mobility/uav/uav.h"
#include <math.h>
#include <cmath>
#include "LIMoSim/world/matrix3d.h"

namespace LIMoSim {

UAV_Locomotion::UAV_Locomotion(std::string _agentId):
    Locomotion(_agentId, name())
{
    m_orientation_acceleration_max_radps2 = Orientation3d(10,10,20);
    m_orientation_velocity_max_radps = Orientation3d(10,10,30);

    m_brake_acceleration_max_mps2 = Vector3d(13,13,13);
    m_acceleration_components_coeff_max = Vector3d(1,1,1);
    m_velocity_components_coeff_max = Vector3d(1,1,1);
}

std::string UAV_Locomotion::name()
{
    return "UAV_Locomotion";
}

UAV *UAV_Locomotion::getAgent()
{
    return dynamic_cast<UAV*>(Locomotion::getAgent());
}

Vector3d UAV_Locomotion::getBrakeAccelerationMax()
{
    return m_brake_acceleration_max_mps2;
}

Vector3d UAV_Locomotion::getAccelerationComponentsMax()
{
    return m_acceleration_components_coeff_max.normed() * getAccelerationMax();
}

Vector3d UAV_Locomotion::getVelocityComponentsMax()
{
    return m_velocity_components_coeff_max.normed() * getVelocityMax();
}

void UAV_Locomotion::setBrakeAccelerationMax(Vector3d _brakeAccelerationMax_mps2)
{
    m_brake_acceleration_max_mps2 = _brakeAccelerationMax_mps2;
}

void UAV_Locomotion::setAccelerationComponentsCoeffMax(Vector3d _accelerationComponentsCoeffMax)
{
    m_acceleration_components_coeff_max = _accelerationComponentsCoeffMax;
}

void UAV_Locomotion::setVelocityComponentsCoeffMax(Vector3d _velocityComponentsCoeffMax)
{
    m_velocity_components_coeff_max = _velocityComponentsCoeffMax;
}

LIMoSim::Vector3d UAV_Locomotion::constrainAcceleration(LIMoSim::Vector3d _acc)
{
    // no handling of null accelerations
    if (!_acc.norm()) {
        return _acc;
    }

    Vector3d acc = Locomotion::constrainAcceleration(_acc);
    return acc;
//    Vector3d accInAgentSpace = projectInAgentSpace(acc);
//    Vector3d accComponentsMax = getAccelerationComponentsMax();
//    Vector3d accInAgentSpaceCompNormedAbs = Vector3d::cMin(
//                accInAgentSpace.cAbs(),
//                accComponentsMax
//                );
//    Vector3d accInAgentSpaceCompNormed = Vector3d::cMult(
//                accInAgentSpace.sign(),
//                accInAgentSpaceCompNormedAbs
//                );
//    Vector3d accCompNormed = fromAgentSpace(accInAgentSpaceCompNormed);
//    return accCompNormed;
}


LIMoSim::Vector3d UAV_Locomotion::constrainVelocity(LIMoSim::Vector3d _velocity)
{
    if (_velocity.norm() < 1e-2) {
        return _velocity;
    }

    Vector3d baseNormVelocity = Locomotion::constrainVelocity(_velocity);
    return baseNormVelocity;
//    Vector3d velocityInAgentSpace = projectInAgentSpace(baseNormVelocity);
//    Vector3d velocityComponentsMax = getVelocityComponentsMax();
//    Vector3d velocityInAgentSpaceCompNormedAbs = Vector3d::cMin(
//                velocityInAgentSpace.cAbs(),
//                velocityComponentsMax
//                );
//    Vector3d velocityInAgentSpaceCompNormed = Vector3d::cMult(
//                velocityInAgentSpace.sign(),
//                velocityInAgentSpaceCompNormedAbs
//                );
//    Vector3d velocityCompNormed = fromAgentSpace(velocityInAgentSpaceCompNormed);
//    return velocityCompNormed;

}

LIMoSim::Steering UAV_Locomotion::normalizeSteering(const LIMoSim::Steering &_steering)
{
    Steering baseNormalizedSteering = Locomotion::normalizeSteering(_steering);
    Vector3d accComponentsMax = getAccelerationComponentsMax();
    Steering normalizedSteering = baseNormalizedSteering;
    normalizedSteering.position = Vector3d::cMin(baseNormalizedSteering.position, accComponentsMax);
    // return normalizedSteering;
    return baseNormalizedSteering;
}

LocomotionUpdate UAV_Locomotion::apply(Steering steering, double _timeDelta_s)
{
    //TODO: Move this block in UAV_Locomotion specialization for DeliveryUAV
    std::string behaviorName = getAgent()->getModel()->getBehaviorName();

    if (getAgent()->getModel()->agentIsCarried()) {
        Vehicle* carryingAgent = getAgent()->getModel()->getCarryingAgent();
        Vector3d nextPos = carryingAgent->getPosition();

//        Energy::EnergyMonitor::getInstance()->saveIncrementalUpdate(getAgent()->getId(), _timeDelta_s, 0);
        // TODO: orientation must be superposed.
        return LocomotionUpdate(carryingAgent->getOrientation(), Orientation3d(),nextPos,Vector3d(), Vector3d());
    }

    // compute next position as consequence of corrective acceleration
    std::vector<Vector3d> nextPosAndVelocity = applyPositionSteering(steering.position, _timeDelta_s);

    // update orientation
    VectorPair nextOrientation = applyOrientationSteering(steering.orientation, _timeDelta_s);

    //std::cout << "next position: " << nextPosAndVelocity.at(0).toString().c_str() << std::endl;

    // register energy consumption for step
//    double energy = m_quadrotorEnergyModel.step(_timeDelta_s, steering.position, steering.orientation);
//    Energy::EnergyMonitor::getInstance()->saveIncrementalUpdate(getAgent()->getId(), _timeDelta_s, energy);

    return LocomotionUpdate(nextOrientation.first, nextOrientation.second, nextPosAndVelocity.at(0), nextPosAndVelocity.at(1), nextPosAndVelocity.at(2));
}

VectorPair UAV_Locomotion::applyOrientationSteering(Orientation3d _orientationSteering, double _timeDelta_s)
{
    // pitch acc. and vel. constraints
    if (abs(_orientationSteering.pitch()) > getOrientationAccelerationMax().pitch()) {
        _orientationSteering.setPitch(getOrientationAccelerationMax().pitch() * _orientationSteering.pitch() / abs(_orientationSteering.pitch()));
    }
    double nextPitchVelocity = getAgent()->getOrientationVelocity().pitch() + _orientationSteering.pitch()*_timeDelta_s;
    if (nextPitchVelocity > getOrientationVelocityMax().pitch()) {
        nextPitchVelocity = getOrientationVelocityMax().pitch() * nextPitchVelocity / abs(nextPitchVelocity);
    }

    // roll acc. and vel. constraints
    if (abs(_orientationSteering.roll()) > getOrientationAccelerationMax().roll()) {
        _orientationSteering.setRoll(getOrientationAccelerationMax().roll() * _orientationSteering.roll() / abs(_orientationSteering.roll()));
    }
    double nextRollVelocity = getAgent()->getOrientationVelocity().roll() + _orientationSteering.roll()*_timeDelta_s;
    if (nextRollVelocity > getOrientationVelocityMax().roll()) {
        nextRollVelocity = getOrientationVelocityMax().roll() * nextRollVelocity / abs(nextRollVelocity);
    }

    // yaw acc. and vel. constraints
    if (abs(_orientationSteering.yaw()) > getOrientationAccelerationMax().yaw()) {
        _orientationSteering.setYaw(getOrientationAccelerationMax().yaw() * _orientationSteering.yaw() / abs(_orientationSteering.yaw()));
    }
    double nextYawVelocity = getAgent()->getOrientationVelocity().yaw() + _orientationSteering.yaw()*_timeDelta_s;
    if (nextYawVelocity > getOrientationVelocityMax().yaw()) {
        nextYawVelocity = getOrientationVelocityMax().yaw() * nextYawVelocity / abs(nextYawVelocity);
    }

    Orientation3d nextOrientation = getAgent()->getOrientation() +
            getAgent()->getOrientationVelocity() * _timeDelta_s +
            _orientationSteering * _timeDelta_s * _timeDelta_s;

    nextOrientation.x = std::fmod(nextOrientation.x, 360.0);
    nextOrientation.y = std::fmod(nextOrientation.y, 360.0);
    nextOrientation.z = std::fmod(nextOrientation.z, 360.0);

    return std::make_pair(
                nextOrientation,
                Orientation3d(nextPitchVelocity, nextRollVelocity, nextYawVelocity)
                );
}

std::vector<Vector3d> UAV_Locomotion::applyPositionSteering(Vector3d _positionSteering, double _timeDelta_s)
{
    // norm the steering acceleration with regard to max acceleration
    _positionSteering = constrainInducedBraking(constrainAcceleration(_positionSteering));
    Vector3d nextAcceleration = _positionSteering;


    // update speed
    Vector3d nextVelocity = (_positionSteering * _timeDelta_s + getAgent()->getVelocity());
    nextVelocity = constrainVelocity(nextVelocity);

    // compute next position as consequence of corrective acceleration
    Vector3d nextPos = _positionSteering*0.5*_timeDelta_s*_timeDelta_s + getAgent()->getVelocity()*_timeDelta_s + getAgent()->getPosition();

    return std::vector<Vector3d>({nextPos, nextVelocity, nextAcceleration});
}

Vector3d UAV_Locomotion::computeInducedBraking(Vector3d _acc)
{
    return getAgent()->getAcceleration() - _acc;
}

Vector3d UAV_Locomotion::constrainInducedBraking(Vector3d _acc)
{
    Vector3d inducedBraking = computeInducedBraking(_acc);
    Vector3d constrainedInducedBraking = Vector3d::cMin(inducedBraking, getBrakeAccelerationMax());
    Vector3d accWithConstrainedInducedBraking = _acc - inducedBraking + constrainedInducedBraking;
    return accWithConstrainedInducedBraking;
}

Vector3d UAV_Locomotion::projectInAgentSpace(Vector3d _vec)
{
    Vector3d agentDirection = getDirection();

    Vector3d u, v, w;
    u = agentDirection;
    v = agentDirection.rotateLeft();
    w = Vector3d::cross(u,v);

    Matrix3d fromAgentPerspective (u,v,w);
    // matrix should always be inversible
    Matrix3d toAgentPerspective = fromAgentPerspective.inv();

    Vector3d agentSpaceProjection = toAgentPerspective * _vec;
    return agentSpaceProjection;
}

Vector3d UAV_Locomotion::fromAgentSpace(Vector3d _vec)
{
    Vector3d agentDirection = getDirection();

    Vector3d u, v, w;
    u = agentDirection;
    v = agentDirection.rotateLeft();
    w = Vector3d::cross(u,v);

    Matrix3d fromAgentPerspective (u,v,w);

    Vector3d original = fromAgentPerspective * _vec;
    return original;
}

Vector3d UAV_Locomotion::getDirection()
{
    Vector3d velocity = getAgent()->getVelocity();
    if (!velocity.norm()){
        Orientation3d currentOrientation = getAgent()->getOrientation();
        Vector3d direction = Vector3d::fromSphere(
                    currentOrientation.pitch(),
                    currentOrientation.yaw(),
                    1.0
                    ).truncated(4);

        return direction;
    }

    return velocity.normed();
}

} // namespace LIMoSim
