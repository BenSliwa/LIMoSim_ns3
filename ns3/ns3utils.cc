#include "ns3utils.h"

#include "ui/data/uidatamanager.h"
#include "LIMoSim/mobility/external/vehiclenoderegistry.h"

namespace LIMoSim {

namespace NS3 {

using namespace ns3;


bool logAnimationWarnings = false;

Vector3d toLIMoSimVector(Vector _ns3Vector)
{
    return Vector3d(_ns3Vector.x, _ns3Vector.y, _ns3Vector.z);
}

Vector toNS3Vector(Vector3d _LIMoSimVector)
{
    return Vector(_LIMoSimVector.x, _LIMoSimVector.y, _LIMoSimVector.z);
}


VehicleUIProps::VehicleUIProps(std::string _color):
    color(_color)
{}

VehicleProperties::VehicleProperties(Vector _position,
        VehicleType _type,
        std::string _id):
    position(_position),
    type(_type),
    id(_id),
    ui (VehicleUIProps("white"))
{}

VehicleProperties::VehicleProperties(Vector _position,
                                    VehicleType _type,
                                    std::string _id,
                                     VehicleUIProps _ui, Callback<Behavior *> _behaviorFactory):
    position(_position),
    type(_type),
    id(_id),
    ui(_ui),
    behaviorFactory(_behaviorFactory)
{}

} // namespace NS3

void UI::AnimationUtils::animateReception(std::string _nodeId)
{
    UI::Data::UIDataManager * uidm = UI::Data::UIDataManager::getInstance();

    auto vehicleNodes = LIMoSim::Mobility::External::VehicleNodeRegistry::getInstance();
    std::string vehicleId = vehicleNodes->getVehicleId(_nodeId);

    if (!vehicleId.empty()) {
        if (!vehicleId.empty() && uidm->isVehicleRegistered(vehicleId)) {
            uidm->getVehicleData(vehicleId)->animateReception();
        } else if (LIMoSim::NS3::logAnimationWarnings) {
            std::cerr << "Warning: cannot animate reception of unregistered vehicle node " << _nodeId << "|" << vehicleId << std::endl;
        }
    } else {
        if (uidm->isStaticNodeRegistered(_nodeId)) {
            uidm->getStaticNodeData(_nodeId)->animateReception();
        } else if (LIMoSim::NS3::logAnimationWarnings) {
            std::cerr << "Warning: cannot animate reception of unregistered static node " << _nodeId << std::endl;
        }
    }
}

void UI::AnimationUtils::animateTransmission(std::string _nodeId)
{
    UI::Data::UIDataManager * uidm = UI::Data::UIDataManager::getInstance();

    auto vehicleNodes = LIMoSim::Mobility::External::VehicleNodeRegistry::getInstance();
    std::string vehicleId = vehicleNodes->getVehicleId(_nodeId);

    if (!vehicleId.empty()) {
        if (uidm->isVehicleRegistered(vehicleId)) {
            uidm->getVehicleData(vehicleId)->animateTransmission();
        } else if (LIMoSim::NS3::logAnimationWarnings) {
            std::cerr << "Warning: cannot animate transmission of unregistered vehicle node " << _nodeId << "|" << vehicleId << std::endl;
        }
    } else{
        if (uidm->isStaticNodeRegistered(_nodeId)) {
            uidm->getStaticNodeData(_nodeId)->animateTransmission();
        } else if (LIMoSim::NS3::logAnimationWarnings) {
            std::cerr << "Warning: cannot animate transmission of unregistered static node " << _nodeId << std::endl;
        }
    }

}

}
