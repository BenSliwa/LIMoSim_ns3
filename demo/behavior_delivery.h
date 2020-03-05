#ifndef BEHAVIOR_DELIVERY_H
#define BEHAVIOR_DELIVERY_H

#include "LIMoSim/mobility/uav/reynolds/behavior.h"
#include <functional>

namespace LIMoSim {
class Building;

namespace mobility {
namespace car {
class DeliveryTruck;
}
}

namespace delivery {
namespace Behaviors {

using namespace mobility::car;

class Behavior_Delivery : public Behavior
{
public:
    Behavior_Delivery(std::string _truckId, bool _predictionEnabled = true, std::string _agentId="");

    std::string getDeliveryTargetId() const;
    double getDeliveryReach() const;
    void startNextDelivery();
    void setDeliveryCompleteCallback(std::function<void(std::string)>);

    // Behavior interface
public:
    Steering apply();

protected:
    double m_followingHeight;
    Vector3d m_targetCoordinates;
    double m_secureHeight;

    double distance2D(Vector3d a, Vector3d b);

    void clearDeliveryTargetId();
    DeliveryTruck *getTruck();
    bool getPredictionEnabled();
    std::string getDeliveryTargetId();

    void setTruckId(std::string _id);
    void setPredictionEnabled(bool _enabled);
    void setAgentId(std::string _agentId);

private:
    std::string m_truckId;
    std::string m_deliveryTargetId;
    Building* m_deliveryTarget;
    bool m_predictionEnabled;
    double m_uavRangeMax = 2500;
    std::function<void(std::string)> m_deliveryCompleteCallback;

    DeliveryTruck* m_truck;
};

} // namespace Behaviors
} // namespace Demo
} // namespace LIMoSim

#endif // BEHAVIOR_DELIVERY_H
