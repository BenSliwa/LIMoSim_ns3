#include "limosimmobilitymodel.h"
#include "ns3utils.h"

// ns3
#include <ns3/log.h>
#include <ns3/enum.h>
#include <ns3/string.h>

// limosim
//#include "LIMoSim/mobility/vehicle.h"
#include "LIMoSim/world/vehiclemanager.h"

namespace LIMoSim {

namespace NS3 {

NS_LOG_COMPONENT_DEFINE ("LimoSimMobility");
NS_OBJECT_ENSURE_REGISTERED (LimoSimMobilityModel);

TypeId LimoSimMobilityModel::GetTypeId()
{
    static TypeId tid = TypeId ("LIMoSim::NS3::LIMoSimMobility")
            .SetParent<MobilityModel>()
            .SetGroupName("Mobility")
            .AddConstructor<LimoSimMobilityModel>()
            .AddAttribute("VehicleType",
                          "Type of the vehicle in LIMoSim",
                          EnumValue(-1),
                          MakeEnumAccessor(&LimoSimMobilityModel::m_vehicleType),
                          MakeEnumChecker(0, "Car",
                                          1, "UAV"))
            .AddAttribute("VehicleId",
                          "Id of the vehicle in LIMoSim.",
                          StringValue(""),
                          MakeStringAccessor(&LimoSimMobilityModel::m_vehicleId),
                          MakeStringChecker())
            .AddAttribute("ShapeColor",
                          "Color of the vehicle's shape in LIMoSim.",
                          StringValue(""),
                          MakeStringAccessor(&LimoSimMobilityModel::m_shapeColor),
                          MakeStringChecker())
            .AddAttribute("BehaviorFactory",
                          "Function used to generate initial vehicle behavior",
                          CallbackValue(),
                          MakeCallbackAccessor(&LimoSimMobilityModel::m_behaviorFactory),
                          MakeCallbackChecker())
            ;

    return tid;
}

std::string LimoSimMobilityModel::GetVehicleId()
{
    return m_vehicleId;
}

void LimoSimMobilityModel::SetVehicleId(std::string _vehicleId)
{
    m_vehicleId = _vehicleId;
}

Vehicle *LimoSimMobilityModel::GetVehicle() const
{
    return VehicleManager::getInstance()->getVehicle(m_vehicleId);
}

void LimoSimMobilityModel::DoDispose()
{
    ns3::MobilityModel::DoDispose ();
}

void LimoSimMobilityModel::DoInitialize()
{
    MobilityModel::DoInitialize ();
    if (m_initialPosition.GetLength() > 0)
        DoSetPosition(m_initialPosition);
}

Vector LimoSimMobilityModel::DoGetPosition() const
{
    Vehicle* vehicle = GetVehicle();
    if (vehicle) {
        Vector pos =  toNS3Vector(vehicle->getPosition());
        // Do not return position with 0 elevation
        // Required by Mmwave3gppPropagationLoss
        if (pos.z < 1) {
            pos.z = 2;
            return pos;
        }
        return pos;
    } else {
        return Vector();
    }
}

void LimoSimMobilityModel::DoSetPosition(const Vector &position)
{
    // Mobility is handled by LIMoSim
    // But ns3 may set the initial position of agent
    Vehicle* vehicle = GetVehicle();
    if (vehicle) {
        vehicle->setPosition(toLIMoSimVector(position));
    } else {
        m_initialPosition = position;
    }
}

Vector LimoSimMobilityModel::DoGetVelocity() const
{
    Vehicle* vehicle = GetVehicle();
    if (vehicle) {
        return toNS3Vector(vehicle->getVelocity());
    } else {
        return Vector();
    }
}

int64_t LimoSimMobilityModel::DoAssignStreams(int64_t)
{
    return 0;
}

}

}
