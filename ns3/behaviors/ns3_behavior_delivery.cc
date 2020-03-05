#include "ns3_behavior_delivery.h"

#include <ns3/core-module.h>
#include <ns3/tcp-socket-factory.h>
#include <ns3/udp-socket-factory.h>
#include <ns3/udp-client.h>

#include "ns3/strategicmodels/ns3_truckdeliverystrategy.h"

#include "LIMoSim/mobility/uav/uav.h"
#include "LIMoSim/mobility/uav/reynolds/target.h"
#include "LIMoSim/mobility/uav/reynolds/behaviors.h"
#include "LIMoSim/mobility/deliverytruck/deliverytruck.h"
//#include "LIMoSim/mobility/car/strategic/truckdelivery.h"

#include "demo/deliverylistservice.h"
#include "demo/deliverysettingsservice.h"

namespace LIMoSim {
namespace NS3 {
namespace Behaviors {


TypeId NS3_Behavior_Delivery::GetTypeId()
{
    static TypeId tid = TypeId ("NS3_Behavior_Delivery")
            .SetParent<Application> ()
            .AddConstructor<NS3_Behavior_Delivery> ()
            .AddAttribute ("TruckId", "Vehicle id of the truck to be managed.",
                           StringValue (""),
                           MakeStringAccessor (&NS3_Behavior_Delivery::m_truckId),
                           MakeStringChecker())
            .AddAttribute ("usePrediction", "Flag to enable mobility prediction",
                           BooleanValue (false),
                           MakeBooleanAccessor (&NS3_Behavior_Delivery::m_usePrediction),
                           MakeBooleanChecker())
            .AddAttribute ("VehicleId", "Vehicle id of the agent",
                           StringValue (""),
                           MakeStringAccessor (&NS3_Behavior_Delivery::m_vehicleId),
                           MakeStringChecker())
    ;
    return tid;
}

NS3_Behavior_Delivery::NS3_Behavior_Delivery():
    Behavior_Delivery ("",false,""),
//    m_delivery(Behavior_Delivery("", false, "")),
    m_dock(Behavior_Dock("","")),
    m_state(DOCKED)
{
    this->setName("NS3_Delivery");
}

NS3_Behavior_Delivery::NS3_Behavior_Delivery(std::string _truckId, bool _predictionEnabled, std::string _agentId):
    Behavior_Delivery ("",false,""),
//    m_delivery(Behavior_Delivery(_truckId, _predictionEnabled, _agentId)),
    m_dock(Behavior_Dock(_truckId,_agentId)),
    m_truckId(_truckId),
    m_state(DOCKED)
{    
    this->setName("NS3_Delivery");
}

NS3_Behavior_Delivery::~NS3_Behavior_Delivery()
{

}

void NS3_Behavior_Delivery::initializeMobility()
{
//    m_delivery = Behavior_Delivery(m_truckId, m_usePrediction, m_vehicleId);
//    m_delivery.setDeliveryCompleteCallback([&](std::string _id) {
//        m_state = RETURNING;
//        std::cout <<  "Uav delivery complete ("<< _id <<"): state updated to returning" << std::endl;
//    });
//    m_dock = Behavior_Dock(m_truckId, m_vehicleId);
//    setAgent(m_vehicleId);
}

void NS3_Behavior_Delivery::Setup(std::string _truckId, bool _predictionEnabled, std::string _agentId)
{
    this->setTruckId(_truckId);
    this->setPredictionEnabled(_predictionEnabled);
    this->setAgentId(_agentId);
    this->setDeliveryCompleteCallback([&](std::string _id) {
        m_state = RETURNING;
        std::cout <<  "Uav delivery complete ("<< _id <<"): state updated to returning" << std::endl;
    });
    m_dock = Behavior_Dock(m_truckId, m_vehicleId);
}

void NS3_Behavior_Delivery::setCommunicationsEnabled(bool _enabled)
{
    m_communicationsEnabled = _enabled;
}

void NS3_Behavior_Delivery::setTruckDeliveryStrategy(NS3_TruckDeliveryStrategy  *_truckDeliveryStrategy)
{
    m_truckDeliveryStrategy = _truckDeliveryStrategy;
}

UavDeliveryState NS3_Behavior_Delivery::getState()
{
    return m_state;
}

Behavior_Dock* NS3_Behavior_Delivery::getDockBehavior()
{
    return &m_dock;
}

DroneInteractionMode NS3_Behavior_Delivery::getDroneInteractionMode()
{
    const delivery::DeliverySettingVariant* droneInteractionMode = delivery::DeliverySettingsService::getInstance()
            ->getDeliverySetting(
                delivery::DeliverySettingsService::DeliverySettingKey::DRONE_INTERACTION_MODE
                );
    if(std::get<int>(*droneInteractionMode) == static_cast<int>(DroneInteractionMode::ONSITE)) {
        return DroneInteractionMode::ONSITE;
    } else if(std::get<int>(*droneInteractionMode) == static_cast<int>(DroneInteractionMode::ENROUTE)){
        return DroneInteractionMode::ENROUTE;
    } else {
        std::cerr << "NS3_Behavior_Delivery::getDroneInteractionMode: unknown drone interaction mode. using default (ONSITE)" << std::endl;
        return DroneInteractionMode::ONSITE;
    }
}

void NS3_Behavior_Delivery::receiveDirectMsg(std::string _msg)
{
    receiveMsg(_msg, 0);
}

void NS3_Behavior_Delivery::onDockingComplete()
{
    if (m_state == DOCKED) {
        this->sendMsg("docked");
//        this->disableReportSending();
    } else {
        // unforeseen case
        std::cout << "NS3_Behavior_Delivery::onDockingComplete  unforeseen case" << std::endl;
    }
}

void NS3_Behavior_Delivery::onDockingTriggered()
{
    if (m_state == RETURNING) {
        m_state = DOCKING;
        std::cout <<  "Uav state changed from RETURNING to DOCKING" << std::endl;
    }
}

void NS3_Behavior_Delivery::onLaunchingComplete()
{
    if (m_state == LOADING) {
        m_state = DELIVERING;
        // notify the mobility model the uav is not being carried anymore
        getAgent()->getModel()->unsetCarrying();
        this->sendMsg("delivering");
        this->startNextDelivery();
//        this->enableReportSending();
    } else {
        // unforeseen case
        std::cout << "NS3_Behavior_Delivery::onLaunchingComplete  unforeseen case" << std::endl;
    }
}

bool NS3_Behavior_Delivery::onLaunchTriggered()
{
    if (m_state == DOCKED) {
        m_state = LOADING;
        int launchDelay = std::get<int>(*delivery::DeliverySettingsService::getInstance()->getDeliverySetting(
                    delivery::DeliverySettingsService::DeliverySettingKey::UAV_LAUNCH_DELAY
                    ));
        Simulator::Schedule(Seconds(launchDelay), &NS3_Behavior_Delivery::onLaunchingComplete, this);
        return true;
    } else {
        // unhandled case
        std::cout << "NS3_Behavior_Delivery::onLaunchTriggered  unforeseen case" << std::endl;
        return false;
    }
}

void NS3_Behavior_Delivery::onDeliveryTargetReached(std::string _deliveryTarget)
{
    if (m_state == DELIVERING) {
        m_state = UNLOADING;
        std::cout <<  "UAV delivery target reached ("
                   << _deliveryTarget
                   <<"): state updated to UNLOADING"
                  << std::endl;
        int unloadingDelay = std::get<int>(*delivery::DeliverySettingsService::getInstance()->getDeliverySetting(
                    delivery::DeliverySettingsService::DeliverySettingKey::UAV_UNLOADING_TIME
                    ));
        Simulator::Schedule(Seconds(unloadingDelay), &NS3_Behavior_Delivery::onUnloadingComplete, this, _deliveryTarget);
    } else {
        // unhandled case
        std::cout << "NS3_Behavior_Delivery::onDeliveryTargetReached  unforeseen case" << std::endl;
    }
}

void NS3_Behavior_Delivery::onUnloadingComplete(std::string _deliveryTarget)
{
    if (m_state == UNLOADING) {
        m_state = RETURNING;
        clearDeliveryTargetId();
        std::cout <<  "UAV unloading complete ("
                   << _deliveryTarget
                   <<"): state updated to RETURNING"
                  << std::endl;
    } else {
        // unhandled case
        std::cout << "NS3_Behavior_Delivery::onUnloadingComplete  unforeseen case" << std::endl;
    }
}

Steering NS3_Behavior_Delivery::apply()
{
    // Choose which behavior is to be adopted
    double distanceToTruck = (getAgent()->getPosition() - Target(m_truckId).getPosition()).norm();

    if (m_state == DOCKED || m_state == LOADING || m_state == DOCKING) {
        if (m_state == DOCKING && distanceToTruck <= 5) {            
            std::cout <<  "UAV state changed from DOCKING to DOCKED" << std::endl;
            m_state = DOCKED;
            // notify mobility model that the UAV is now docked.
            getAgent()->getModel()->setCarryingAgent(m_truckId);
            int dockingDelay = std::get<int>(*delivery::DeliverySettingsService::getInstance()->getDeliverySetting(
                        delivery::DeliverySettingsService::DeliverySettingKey::UAV_DOCKING_DELAY
                        ));
            Simulator::Schedule(Seconds(dockingDelay), &NS3_Behavior_Delivery::onDockingComplete, this);
        }
        return m_dock.apply();
    }

    if (m_state == UNLOADING) {
//        return Behavior_NoOp().apply();
        return Behavior_Arrive(m_targetCoordinates,10, getAgent()->getId()).apply();
    }

    if (m_state == DELIVERING) {
        Vector3d agentPos = getAgent()->getPosition();
        // Check for transistion in UNLOADING
        if ((agentPos - m_targetCoordinates).norm() < 10) {
            // delivery was just completed
            delivery::DeliveryListService::getInstance()->notifyDelivery(getDeliveryTargetId());
            onDeliveryTargetReached(getDeliveryTargetId());
            return Behavior_NoOp().apply();
        }

        // If still far away from target
        if ( (Vector3d(agentPos.x, agentPos.y) -
              Vector3d(m_targetCoordinates.x, m_targetCoordinates.y)).norm() > 20){
            if ( std::abs(agentPos.z - m_secureHeight) > 10.0) {
                // Ascend
                return Behavior_Arrive(Vector3d(agentPos.x, agentPos.y, m_secureHeight),
                                       50,
                                       getAgent()->getId()).apply();
            } else {
                // Move towards delivery target
                auto b = new Behavior_Arrive(m_targetCoordinates,
                                             50,
                                             getAgent()->getId());
                auto s = makeSelfAligned(b)->apply();
                delete b;
                return s;
            }
        }
        else {
            // Descend
            return Behavior_Arrive(m_targetCoordinates,50, getAgent()->getId()).apply();
        }
    }

    if (m_state == RETURNING) {
        Vector3d agentPos = getAgent()->getPosition();
        // Check for transition to DOCKING state
        if (distanceToTruck <=  10) {
            // just reached the truck
            // startNextDelivery();
            // m_state = DOCKING;
            this->sendMsg("docking");
            // std::cout <<  "Uav state changed from RETURNING to DOCKING" << std::endl;
            // return m_dock.apply();
        }
        if (distance2D(agentPos, Target(m_truckId).getPosition()) > 20) {
            if (std::abs(agentPos.z - m_secureHeight) > 10.0) {
                // Ascend
                return Behavior_Arrive(Vector3d(agentPos.x, agentPos.y, m_secureHeight),
                                       50,
                                       getAgent()->getId()).apply();
            } else {
                DroneInteractionMode droneInteractionMode = getDroneInteractionMode();
                if( droneInteractionMode == DroneInteractionMode::ONSITE) {
                    // Move towards truck's next delivery target
//                    std::cout << "NS3_Behavior_Delivery::move (RETURNING) moving towards next truck delivery target" << std::endl;
                    return Behavior_Arrive(getTruck()->getCurrentDeliveryTargetPosition(),50, getAgent()->getId()).apply();
                } else if(droneInteractionMode == DroneInteractionMode::ENROUTE){
                    // Move towards truck
                    return Behavior_FollowAtElevation(m_truckId,m_secureHeight,5,getPredictionEnabled(),getAgent()->getId()).apply();
                }
            }
        } else {
            // Descend to truck
//            std::cout << "NS3_Behavior_Delivery::move (RETURNING) descending to truck" << std::endl;
            return Behavior_FollowAtElevation(m_truckId,m_followingHeight,5,getPredictionEnabled(),getAgent()->getId()).apply();
    //        return Behavior_Pursuit(Target(m_truckId), getAgent()->getId()).apply();
        }
    }

    std::cout << "NS3_Behavior_Delivery::apply unforeseen case" << std::endl;
    return Behavior_NoOp().apply();
}

void NS3_Behavior_Delivery::setAgent(std::string _agentId)
{
    Behavior::setAgent(_agentId);
//    m_delivery.setAgent(_agentId);
    m_dock.setAgent(_agentId);
}

void NS3_Behavior_Delivery::DoDispose()
{
    NS_LOG_FUNCTION_NOARGS ();
    Application::DoDispose ();
}

void NS3_Behavior_Delivery::StartApplication()
{
    NS_LOG_FUNCTION_NOARGS ();
    if (m_communicationsEnabled) {
        m_transciever = ns3::DynamicCast<UdpTransciever>(GetNode()->GetApplication(0));
        m_transciever->setReceiveCallback(MakeCallback(&NS3_Behavior_Delivery::receiveMsg, this));
    }
    getAgent()->getModel()->setCarryingAgent(m_truckId);
}

void NS3_Behavior_Delivery::StopApplication()
{
    NS_LOG_FUNCTION_NOARGS ();
}

void NS3_Behavior_Delivery::receiveMsg(std::string _msg, uint _senderIndex)
{
    std::cout << "(UavDelivery) received message: " << _msg << std::endl;
    bool handlingOk = false;
    if (_msg == "launching") {
         handlingOk = onLaunchTriggered();
    } else if (_msg == "ACK_DOCKING") {
        return onDockingTriggered();
    }


    if (handlingOk) {
        return this->sendMsg("ACK_"+_msg);
    } else {
        return this->sendMsg("NACK_"+_msg);
    }
}

void NS3_Behavior_Delivery::sendMsg(std::string _msg)
{
    if (m_communicationsEnabled) {
        m_transciever->send(_msg);
    } else {
        m_truckDeliveryStrategy->receiveDirectMsg(_msg, this);
    }
}

void NS3_Behavior_Delivery::enableReportSending()
{
    NS_LOG_FUNCTION_NOARGS ();
    Ptr<UdpClient> reportSender = ns3::DynamicCast<UdpClient>(GetNode()->GetApplication(2));
    reportSender->SetStartTime(Simulator::Now() + MilliSeconds(1));
    reportSender->Initialize();
}

void NS3_Behavior_Delivery::disableReportSending()
{
    NS_LOG_FUNCTION_NOARGS ();
    Ptr<Application> reportSender = ns3::DynamicCast<UdpClient>(GetNode()->GetApplication(2));
    reportSender->SetStopTime(Simulator::Now() + MilliSeconds(1));
}

} // namespace Behaviors
} // namespace NS3
} // namespace LIMoSim
