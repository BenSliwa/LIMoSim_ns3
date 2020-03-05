#include "uav.h"

#include "LIMoSim/mobility/uav/reynolds/uav_reynoldsmodel.h"
#include "LIMoSim/mobility/uav/reynolds/locomotionupdate.h"
#include "LIMoSim/mobility/uav/reynolds/behavior.h"

namespace LIMoSim
{

UAV::UAV(const std::string &_id) :
    Vehicle(_id, "UAV")
{
    setModel(new UAV_ReynoldsModel(_id));
    initialize();
}

UAV_ReynoldsModel *UAV::getModel()
{
    return dynamic_cast<UAV_ReynoldsModel*>(Vehicle::getModel());
}

void UAV::setAccelerationMax(double _accMax)
{
    getModel()->getLocomotion()->setAccelerationMax(_accMax);
}

void UAV::setBehavior(Behavior *_behavior)
{
    _behavior->setAgent(this->getId());
    getModel()->setBehavior(_behavior);
}

void UAV::setVelocityMax(double _velMax)
{
    getModel()->getLocomotion()->setVelocityMax(_velMax);
}


/*************************************
 *          PROTECTED METHODS        *
 ************************************/

void UAV::initialize()
{
    Vehicle::initialize();
}

void UAV::handleEvent(Event *_event)
{
    Vehicle::handleEvent(_event);

}

void UAV::move(double _timeDelta_s)
{
    LocomotionUpdate update = getModel()->step(_timeDelta_s);
//    if (rand() * 1.0 / RAND_MAX > 0.7) {
//        double noiseMagnitude = 0 + ((double)rand() / RAND_MAX) * 0.5;
//        update.position = update.position + generateNormedNoiseVector() * noiseMagnitude;
//    }
    setPosition(update.position);
    setVelocity(update.velocity);
    setAcceleration(update.acceleration);
    setOrientation(update.orientation);
    setOrientationVelocity(update.orientationVelocity);
}


/*************************************
 *          PRIVATE METHODS          *
 ************************************/


Vector3d UAV::generateNormedNoiseVector()
{
    return Vector3d(-0.5 + ((double)rand() / RAND_MAX), -0.5 + ((double)rand() / RAND_MAX), -0.5 + ((double)rand() / RAND_MAX)).normed();
}

}
