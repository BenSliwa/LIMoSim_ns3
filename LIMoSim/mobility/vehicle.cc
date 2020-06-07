#include "vehicle.h"

#include "LIMoSim/world/vehiclemanager.h"
#include "LIMoSim/world/localvehiclemanager.h"


namespace LIMoSim
{

Vehicle::Vehicle(const std::string &_id, const std::string &_type) :
    EventHandler(),
    m_id(_id),
    m_type(_type),
    m_lastUpdate_s(0),
    m_updateTimer(nullptr),
    m_updateInterval_s(0.01),
    m_mobilityModel(nullptr),
    moveSpeedUp(1)
{

    m_mobilityDataExporter = new MobilityDataExporter(this, 0.1);
}

Vehicle::~Vehicle()
{
    delete m_mobilityDataExporter;
    if (m_updateTimer){
        deleteEvent(m_updateTimer);
    }
}

void Vehicle::setAcceleration(const Vector3d &_acceleration)
{
    m_acceleration_mps2 = _acceleration;
}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void Vehicle::setPosition(const Vector3d &_position)
{
    m_position = _position;
    auto lvm = getLocalVehicleManager();
    if (lvm) {
        lvm->updateMobilityData();
    }
}

void Vehicle::setOrientation(const Orientation3d &_orientation)
{
    m_orientation = _orientation;
}

void Vehicle::setVelocity(const Vector3d &_velocity)
{
    m_velocity_mps = _velocity;
    auto lvm = getLocalVehicleManager();
    if (lvm) {
        lvm->updateMobilityData();
    }
}

void Vehicle::setOrientationVelocity(const Orientation3d &_orientationVelocity)
{
    m_orientationVelocity_radps = _orientationVelocity;
}

std::string Vehicle::getId()
{
    return m_id;
}

std::string Vehicle::getType()
{
    return m_type;
}

Vector3d Vehicle::getAcceleration()
{
    return m_acceleration_mps2;
}

Vector3d Vehicle::getPosition()
{
    return m_position;
}

Orientation3d Vehicle::getOrientation()
{
    return m_orientation;
}

Vector3d Vehicle::getVelocity()
{
    return m_velocity_mps;
}

Orientation3d Vehicle::getOrientationVelocity()
{
    return m_orientationVelocity_radps;
}

MobilityModel *Vehicle::getModel()
{
    return m_mobilityModel;
}

LocalVehicleManager *Vehicle::getLocalVehicleManager()
{
    return VehicleManager::getInstance()->getLocalVehicleManager(this->getId());
}

void Vehicle::initialize()
{
    m_updateTimer = new Event(m_updateInterval_s, this, "Move");
    scheduleEvent(m_updateTimer);
    m_mobilityDataExporter->start();
}

void Vehicle::handleEvent(Event *_event)
{
    if (_event->getInfo() == m_updateTimer->getInfo()) {
        handleMoveEvent(_event);
    } else {
        delete _event;
    }
}

void Vehicle::handleMoveEvent(Event *_event)
{
    double timeDelta_s = _event->getTimestamp() - m_lastUpdate_s;
    move(timeDelta_s * moveSpeedUp);
    m_lastUpdate_s = _event->getTimestamp();

    //
    scheduleEvent(_event, m_updateInterval_s);
}

void Vehicle::setModel(MobilityModel *_mobilityModel)
{
    if(m_mobilityModel) {
        delete m_mobilityModel;
    }
    m_mobilityModel = _mobilityModel;
}

}
