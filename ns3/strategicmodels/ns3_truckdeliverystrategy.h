#ifndef NS3_TRUCKDELIVERYSTRATEGY_H
#define NS3_TRUCKDELIVERYSTRATEGY_H

#include "LIMoSim/mobility/car/strategic/truckdelivery.h"

#include <ns3/application.h>
#include <ns3/socket.h>

#include "ns3/applications/udptransciever.h"

//#include "ns3/behaviors/ns3_behavior_delivery.h"


namespace LIMoSim {

using namespace  mobility::car;
using namespace  world::utils;

namespace NS3 {

using namespace Applications;

namespace Behaviors {
    class NS3_Behavior_Delivery;
}


namespace StrategicModels {

using namespace ns3;

class NS3_TruckDeliveryStrategy: public Application, public TruckDelivery
{
public:
    static TypeId GetTypeId (void);
    NS3_TruckDeliveryStrategy();
    virtual ~NS3_TruckDeliveryStrategy();

    void initializeMobility(std::string _truckId);
    void setCommunicationsEnabled(bool _communicationsEnabled);
    void setUavDeliveryBehaviors(std::vector<Behaviors::NS3_Behavior_Delivery*> _deliveryBehaviors);

    void launchUAVOnSite(Endpoint *_endpoint);
    void launchUAVEnRoute(EnRouteDelivery _enRouteDelivery);
    void completeUnloading(Endpoint *_endpoint);
    void onLaunchUavComplete();
    void onRecoveringComplete();
    void resumeDelivery(double _delay = 0);

    void receiveDirectMsg(std::string _msg, Behaviors::NS3_Behavior_Delivery* _sender);


    // StrategicModel interface
    void handleDeliveryTargetReached(std::string _targetId, Endpoint *_endpoint);

    // Application interface
protected:
    virtual void DoDispose (void);
    DeliveryTruck* getTruckById(std::string _truckId);
    int getNextAvailableUavIndex();

private:
    virtual void StartApplication (void);
    virtual void StopApplication (void);

    // Parcel Delivery Communications
     void receiveMsg (std::string _msg, uint _senderIndex);
     void sendMsg (std::string _msg, uint _uavIndex);
     int getUnloadingDelay();

private:
    std::string m_truckId;

    Ipv4Address     m_destAddr;
    uint16_t        m_destPort;
    Ptr<Socket>     m_socket;    
    Ptr<UdpTransciever> m_transciever;
    std::vector<Behaviors::NS3_Behavior_Delivery*> m_UavDeliveryBehaviors;

    uint m_pendingLaunches = 0;
    uint m_pendingRecoveries = 0;

    uint m_requiredUavs = 0;

    bool m_communicationsEnabled;
    Endpoint* m_currentEndpoint = nullptr;
    TruckDeliveryState m_preRecoveryState = NONE;
};

} // namespace StrategicModels
} // namespace NS3
} // namespace LIMoSim

#endif // NS3_TRUCKDELIVERYSTRATEGY_H
