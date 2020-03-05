#include "target.h"
#include "LIMoSim/mobility/uav/uav.h"
#include "LIMoSim/world/vehiclemanager.h"

namespace LIMoSim {

Target::Target(Vector3d _point):
    m_point(_point),
    m_vehicleId(""),
    m_isMobile(false)
{

}

Target::Target(std::string _vehicleId):
    m_vehicleId(_vehicleId),
    m_isMobile(true)
{

}

Target::Target(const Target &_ref):
    m_point(_ref.m_point),
    m_vehicleId(_ref.m_vehicleId),
    m_isMobile(_ref.m_isMobile)
{

}

bool Target::isMobile()
{
    return m_isMobile;
}

Vector3d Target::getPosition()
{
    if (m_isMobile) {
        return VehicleManager::getInstance()->getVehicle(m_vehicleId)->getPosition();
    } else {
        return m_point;
    }
}

Vector3d Target::getVelocity()
{
    if (m_isMobile) {
        return VehicleManager::getInstance()->getVehicle(m_vehicleId)->getVelocity();
    } else {
        return Vector3d();
    }
}

std::string Target::getVehicleId()
{
    return m_vehicleId;
}

void Target::set(Vector3d _point)
{
    m_point = _point;
    m_isMobile = false;
}

void Target::set(std::string _vehicleId)
{
    m_vehicleId = _vehicleId;
    m_isMobile = true;
}

} // namespace LIMoSim
