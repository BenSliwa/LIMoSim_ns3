#ifndef NS3_BEHAVIOR_DELIVERY_H
#define NS3_BEHAVIOR_DELIVERY_H

#include <ns3/application.h>
#include <ns3/socket.h>

#include "ns3/applications/udptransciever.h"

#include "LIMoSim/mobility/uav/reynolds/behavior.h"

#include "demo/behavior_dock.h"
#include "demo/behavior_delivery.h"
#include "demo/deliverytypedefs.h"

namespace LIMoSim {
namespace NS3 {

using namespace Applications;

namespace StrategicModels {
    class NS3_TruckDeliveryStrategy;
}
using namespace StrategicModels;

namespace Behaviors {

using namespace ns3;
using namespace delivery::Behaviors;
using namespace delivery::typedefs;

enum UavDeliveryState { DOCKED=0, LOADING, DELIVERING, UNLOADING, RETURNING, DOCKING};

class NS3_Behavior_Delivery : public Application, public Behavior_Delivery
{
public:
    static TypeId GetTypeId (void);
    NS3_Behavior_Delivery();
    NS3_Behavior_Delivery(std::string _truckId, bool _predictionEnabled = true, std::string _agentId="");
    virtual ~NS3_Behavior_Delivery();

    void initializeMobility();
    void Setup(std::string _truckId, bool _predictionEnabled, std::string _agentId);
    void setCommunicationsEnabled(bool _enabled);
    void setTruckDeliveryStrategy(NS3_TruckDeliveryStrategy *_truckDeliveryStrategy);
    UavDeliveryState getState();
    Behavior_Dock *getDockBehavior();
    DroneInteractionMode getDroneInteractionMode();
    void receiveDirectMsg(std::string _msg);


    void onDockingComplete();
    void onDockingTriggered();
    void onLaunchingComplete();
    bool onLaunchTriggered();
    void onDeliveryTargetReached(std::string _deliveryTarget);
    void onUnloadingComplete(std::string _deliveryTarget);
    // Behavior interface
public:
    Steering apply();
    virtual void setAgent(std::string _agentId);

    // Application interface
protected:
    virtual void DoDispose (void);
private:
    virtual void StartApplication (void);
    virtual void StopApplication (void);

    // Parcel Delivery Communications
     void receiveMsg (std::string _msg, uint _senderIndex);
     void sendMsg (std::string _msg);

     //
     void enableReportSending();
     void disableReportSending();

private:
//    Behavior_Delivery m_delivery;
    Behavior_Dock m_dock;
    std::string m_truckId;
    std::string m_vehicleId;
    bool m_usePrediction;

    UavDeliveryState m_state;

    bool m_communicationsEnabled;
    Ptr<UdpTransciever> m_transciever;
    NS3_TruckDeliveryStrategy * m_truckDeliveryStrategy;

};

} // namespace Behaviors
} // namespace NS3
} // namespace LIMoSim

#endif // NS3_BEHAVIOR_DELIVERY_H
