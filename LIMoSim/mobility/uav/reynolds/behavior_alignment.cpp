#include "behavior_alignment.h"

#include "LIMoSim/mobility/uav/uav.h"
#include "LIMoSim/world/vehiclemanager.h"

namespace LIMoSim {

Behavior_Alignment::Behavior_Alignment(double _searchRadius, std::string _agentId):
    Behavior("Alignment", _agentId),
    m_searchRadius(_searchRadius)
{

}

Behavior_Alignment::Behavior_Alignment(std::string _referenceAgentId, std::string _agentId):
    Behavior("Alignment", _agentId),
    m_searchRadius(0),
    m_referenceAgentId(_referenceAgentId)
{

}

Steering Behavior_Alignment::apply()
{
    Vector3d desiredVelocity;
    Vector3d positionSteering;

    // Align with respect to reference agent's direction
    if (getReferenceAgent()) {
        MobilityDataMap mobilityReports = getAgent()->getLocalVehicleManager()->getMobilityData();
        auto it = mobilityReports.find(m_referenceAgentId);

        // By missing mobility data of reference agent take no action
        if (it == mobilityReports.end()) {
            return Steering();
        }
        else {
            positionSteering = Vector3d();
            MobilityData referenceAgentMobility = mobilityReports.at(m_referenceAgentId);
            // Align on the reference agent's speed if not negligible
            if (referenceAgentMobility.velocity.norm() > 1e-1) {
                desiredVelocity = referenceAgentMobility.velocity;
                positionSteering = desiredVelocity - getAgent()->getVelocity();
            }
            // Else align on the reference agent's orientation
            else {
                desiredVelocity = Vector3d::fromSphere(
                            referenceAgentMobility.orientation.pitch(),
                            referenceAgentMobility.orientation.yaw(),
                            1.0
                            ).truncated(4);
            }
        }
    }
    // Align with respect to mean direction of surrounding vehicles
    else {
        MobilityDataMap nearbyUAVs = findNearByUAVs();
        Vector3d meanVelocity;

        if (!nearbyUAVs.size()) {
            // stop movement
            return Steering(Orientation3d(), getAgent()->getVelocity() * -2 );
        }

        // compute mean position
        for (auto it = nearbyUAVs.begin(); it != nearbyUAVs.end(); it++) {
            MobilityData data = it->second;
            if (data.velocity.norm() > 1e-1) {
                meanVelocity = meanVelocity + data.velocity;
            } else {
                meanVelocity = meanVelocity + Vector3d::fromSphere(
                            data.orientation.pitch(),
                            data.orientation.yaw(),
                            (int) nearbyUAVs.size()
                            ).truncated(4);
            }
        }

        // mean velocity is the desired velocity
        meanVelocity = meanVelocity / (double)nearbyUAVs.size();

        desiredVelocity = getAgent()->getModel()->getLocomotion()->constrainVelocity(meanVelocity);
        positionSteering = desiredVelocity - getAgent()->getVelocity();
    }

    Orientation3d currentOrientation = getAgent()->getOrientation();
    Orientation3d currentOrientationVelocity = getAgent()->getOrientationVelocity();
    Orientation3d desiredOrientationVelocity = computeDesiredOrientationVelocity(desiredVelocity, currentOrientation);
    Orientation3d orientationSteering = computeOrientationArrivalSteering(desiredOrientationVelocity, currentOrientationVelocity);

    return Steering(orientationSteering, positionSteering);
}

UAV *Behavior_Alignment::getReferenceAgent()
{
    if (m_referenceAgentId.empty()) {
        return nullptr;
    } else {
        return dynamic_cast<UAV*>(VehicleManager::getInstance()->getVehicle(m_referenceAgentId));
    }
}

std::map<std::string, MobilityData> Behavior_Alignment::findNearByUAVs()
{
    auto mobilityDataReport = getAgent()->getLocalVehicleManager()->getMobilityData();

    std::map<std::string, MobilityData> nearby;

    for (auto it : mobilityDataReport) {
        MobilityData data = it.second;
        if ( (data.position - getAgent()->getPosition()).norm() <= m_searchRadius ){
            nearby.insert((nearby.end(), it));
        }
    }

    return nearby;
}

} // namespace LIMoSim
