#include "ns3_truckdeliverystrategy.h"

#include <algorithm>

#include <ns3/core-module.h>
#include <ns3/tcp-socket-factory.h>
#include <ns3/udp-socket-factory.h>

#include "ns3/behaviors/ns3_behavior_delivery.h"

#include "LIMoSim/world/vehiclemanager.h"
#include "demo/deliverysettingsservice.h"
#include "demo/deliverylistservice.h"


#include "LIMoSim/utils/vector.h"

namespace LIMoSim {
namespace NS3 {

namespace StrategicModels {

TypeId NS3_TruckDeliveryStrategy::GetTypeId()
{
    static TypeId tid = TypeId ("NS3_TruckDeliveryStrategy")
      .SetParent<Application> ()
      .AddConstructor<NS3_TruckDeliveryStrategy> ()
      .AddAttribute ("TruckId", "Vehicle id of the truck to be managed.",
                     StringValue (""),
                     MakeStringAccessor (&NS3_TruckDeliveryStrategy::m_truckId),
                     MakeStringChecker())
    ;
    return tid;
}

NS3_TruckDeliveryStrategy::NS3_TruckDeliveryStrategy():
    TruckDelivery (nullptr)
{
    NS_LOG_FUNCTION_NOARGS ();
}

NS3_TruckDeliveryStrategy::~NS3_TruckDeliveryStrategy()
{
    NS_LOG_FUNCTION_NOARGS ();
}

void NS3_TruckDeliveryStrategy::initializeMobility(std::string _truckId)
{
    if (!_truckId.empty()){
        this->SetAttribute("TruckId", StringValue(_truckId));
    }
    DeliveryTruck *truck = getTruckById(m_truckId);
    this->p_car = truck;
    this->p_truck = truck;
    //    this->initialize();
}

void NS3_TruckDeliveryStrategy::setCommunicationsEnabled(bool _communicationsEnabled)
{
    m_communicationsEnabled = _communicationsEnabled;
}

void NS3_TruckDeliveryStrategy::setUavDeliveryBehaviors(std::vector<Behaviors::NS3_Behavior_Delivery *> _deliveryBehaviors)
{
    m_UavDeliveryBehaviors = _deliveryBehaviors;
}

void NS3_TruckDeliveryStrategy::launchUAVOnSite(Endpoint *_endpoint)
{
    uint endpointIdx = std::distance(m_tspd.endpoints.begin(), std::find(m_tspd.endpoints.begin(), m_tspd.endpoints.end(), _endpoint));
    uint routeIndex = utils::vector::indexOf(m_tspd.truckRoute, endpointIdx);

    if (routeIndex == 0 && this->isFirstDeliveryCompleted()) {
        std::cout << "no launching scheduled on start node revisit" << std::endl;
        return;
    }

    std::cout << "launching uav on site:" << std::endl;
    std::cout << "current endpoint index:" << endpointIdx << std::endl;
    std::cout << "scheduled launches: " << m_tspd.launchRegistry.at(endpointIdx).size() << std::endl;
    std::cout << "availabe drones:" << getAvailableDroneCount() << std::endl;
    if (
            m_tspd.launchRegistry.count(endpointIdx) &&
            m_tspd.launchRegistry.at(endpointIdx).size() > 0
        ) {
        m_state = LAUNCHING;
        uint scheduledLaunches =  m_tspd.launchRegistry.at(endpointIdx).size();
        if (scheduledLaunches > getAvailableDroneCount()) {
            m_requiredUavs = scheduledLaunches - getAvailableDroneCount();
            Simulator::Schedule(Seconds(2), &NS3_TruckDeliveryStrategy::launchUAVOnSite, this, _endpoint);
            std::cout << "launching reported due to unsufficient drones" << std::endl;
            return;
        }
        for (
             uint completedLaunches = 0;
             completedLaunches < scheduledLaunches;
             completedLaunches++
             )
        {
            // Should always be one available if we came this far
            uint uavIndex = static_cast<uint>(getNextAvailableUavIndex());
            this->sendMsg("launching", uavIndex);
            m_pendingLaunches++;
            decrementDroneCount();
        }
        m_requiredUavs = 0;
        // Getting out of the lauching state requires feedback from UAV
        // that launching is completed
    } else {
        this->resumeDelivery();
    }
}

void NS3_TruckDeliveryStrategy::launchUAVEnRoute(EnRouteDelivery _enRouteDelivery)
{
    Endpoint* nextTruckEndpoint = getNextEndpoint();
    Endpoint * nextTruckEndpointForDelivery = m_tspd.endpoints.at(m_tspd.truckRoute.at(_enRouteDelivery.nextTruckDelivery));

    if (
            nextTruckEndpoint == nextTruckEndpointForDelivery
        ) {
        m_state = LAUNCHING;
        // Should always be one available if we came this far
        uint uavIndex = static_cast<uint>(getNextAvailableUavIndex());
        this->sendMsg("launching", uavIndex);
        m_pendingLaunches++;
        decrementDroneCount();
        // Getting out of the lauching state requires feedback from UAV
        // that launching is completed
    } else {
        this->resumeDelivery();
    }
}

void NS3_TruckDeliveryStrategy::completeUnloading(Endpoint *_endpoint)
{
    if (m_state  == UNLOADING) {
        m_state = STOPPED;
        if (getDroneInteractionMode() == ONSITE) {
            launchUAVOnSite(_endpoint);
        } else {
            this->resumeDelivery();
        }
    } else if (m_state == RECOVERING) {
        m_currentEndpoint = _endpoint;
    } else {
        std::cout << "NS3_TruckDeliveryStrategy::completeUnloading  unhandled case" << std::endl;
    }
}

void NS3_TruckDeliveryStrategy::onLaunchUavComplete()
{
    --m_pendingLaunches;
    if (m_state  == LAUNCHING){
        if (!(m_pendingLaunches)) {
            m_state = STOPPED;
        }
    }
//    if (m_preRecoveryState == LAUNCHING) {
//        m_preRecoveryState = STOPPED;
//    }
    this->resumeDelivery();
}

void NS3_TruckDeliveryStrategy::onRecoveringComplete()
{
    // If there's a current endpoint then unload completion
    // could not complete because of recovery
    if (m_currentEndpoint && m_state == RECOVERING) {
        incrementDroneCount();
        if (!m_pendingRecoveries) {
            m_state = UNLOADING;
            this->completeUnloading(m_currentEndpoint);
            m_currentEndpoint = nullptr;
            if (delivery::DeliveryListService::getInstance()->allDelivered() && getAvailableDroneCount() == p_truck->getDelivererCount()) {
                Simulation::getInstance()->stop();
            }
        }
        std::cout << "NS3_TruckDeliveryStrategy::onRecoveringComplete  next state is: " << getState() << std::endl;
    }
    else if (m_preRecoveryState == UNLOADING) {
        incrementDroneCount();
        if (!m_pendingRecoveries) {
            m_state = UNLOADING;
            m_preRecoveryState = NONE;
            if (delivery::DeliveryListService::getInstance()->allDelivered() && getAvailableDroneCount() == p_truck->getDelivererCount()) {
                Simulation::getInstance()->stop();
            }
        }
        std::cout << "NS3_TruckDeliveryStrategy::onRecoveringComplete  next state is: " << getState() << std::endl;
    } else if  (m_preRecoveryState  == DELIVERING || m_preRecoveryState == STOPPED){        
        incrementDroneCount();
        if (!m_pendingRecoveries) {
            m_state = STOPPED;
            m_preRecoveryState = NONE;
            this->resumeDelivery();
            if (delivery::DeliveryListService::getInstance()->allDelivered() && getAvailableDroneCount() == p_truck->getDelivererCount()) {
                Simulation::getInstance()->stop();
            }
        }
        std::cout << "NS3_TruckDeliveryStrategy::onRecoveringComplete  next state is: " << getState() << std::endl;
    }
    else if (m_preRecoveryState == LAUNCHING) {
        incrementDroneCount();
        if(!m_pendingRecoveries) {
            m_state = m_preRecoveryState;
            m_preRecoveryState = NONE;
            if (!m_pendingLaunches && m_requiredUavs <= 0) {
                m_state = STOPPED;
                this->resumeDelivery();
            }
        }
        std::cout << "NS3_TruckDeliveryStrategy::onRecoveringComplete  next state is: " << getState() << std::endl;
    }
    else {
        std::cout << "NS3_TruckDeliveryStrategy::onRecoveringComplete  unhandled case with state " << getState() << std::endl;
    }
}

void NS3_TruckDeliveryStrategy::resumeDelivery(double _delay)
{
    if (m_state == STOPPED) {
        TruckDelivery::resumeDelivery(_delay);
    }
}

void NS3_TruckDeliveryStrategy::receiveDirectMsg(std::string _msg, Behaviors::NS3_Behavior_Delivery *_sender)
{
    int index = utils::vector::indexOf(m_UavDeliveryBehaviors, _sender);
    if (index != -1) {
        this->receiveMsg(_msg, static_cast<uint>(index));
    }
}

void NS3_TruckDeliveryStrategy::handleDeliveryTargetReached(std::string _targetId, Endpoint  *_endpoint)
{
    if (isFirstDeliveryCompleted()){
        stopDelivery();
        m_state = UNLOADING;
        Simulator::Schedule(Seconds(getUnloadingDelay()), &NS3_TruckDeliveryStrategy::completeUnloading, this, _endpoint);
    } else if(getDroneInteractionMode() == ONSITE) {
        stopDelivery();
        launchUAVOnSite(_endpoint);
    }
    TruckDelivery::handleDeliveryTargetReached(_targetId, _endpoint);
}

void NS3_TruckDeliveryStrategy::DoDispose()
{
    NS_LOG_FUNCTION_NOARGS ();
    Application::DoDispose ();
}

DeliveryTruck *NS3_TruckDeliveryStrategy::getTruckById(std::string _truckId)
{
    Vehicle* vehicle = VehicleManager::getInstance()->getVehicle(_truckId);
    if (vehicle) {
        DeliveryTruck *truck = dynamic_cast<DeliveryTruck*>(vehicle);
        return truck;
    } else {
        std::cerr << "NS3_TruckDeliveryStrategy::getTruckById could not find truck with id "
                  << _truckId << std::endl;
        exit(1100);
    }
}

int NS3_TruckDeliveryStrategy::getNextAvailableUavIndex()
{
    int uavIndex = utils::vector::indexOf(
                m_UavDeliveryBehaviors,
                [](Behaviors::NS3_Behavior_Delivery* _uavBehavior) {
                    return _uavBehavior->getState() == Behaviors::UavDeliveryState::DOCKED;
                }
    );
    return uavIndex;
}

void NS3_TruckDeliveryStrategy::StartApplication()
{
    NS_LOG_FUNCTION_NOARGS ();
    if (m_communicationsEnabled) {
        m_transciever = ns3::DynamicCast<UdpTransciever>(GetNode()->GetApplication(0));
        m_transciever->setReceiveCallback(MakeCallback(&NS3_TruckDeliveryStrategy::receiveMsg, this));
    }
    startFirstDelivery();
}

void NS3_TruckDeliveryStrategy::StopApplication()
{
    NS_LOG_FUNCTION_NOARGS ();
}

void NS3_TruckDeliveryStrategy::receiveMsg(std::string _msg, uint _senderIndex)
{
    std::cout << "(TruckDelivery) received message: " << _msg << std::endl;
    std::cout << "(TruckDelivery) current state: " << m_state << std::endl;
    if (_msg == "docking") {
        this->sendMsg("ACK_DOCKING", _senderIndex);
        if (m_state != RECOVERING) {
            m_preRecoveryState = m_state;
            this->stopDelivery();
        }
        m_pendingRecoveries++;
        m_state = RECOVERING;

    } else if (_msg == "docked") {
        m_pendingRecoveries--;
        std::cout << "(TruckDelivery) pending recoveries:" << m_pendingRecoveries << std::endl;
        onRecoveringComplete();
    }
    else if (_msg == "delivering") {
        if (
                m_state == STOPPED ||
                m_state == LAUNCHING ||
                m_state == UNLOADING ||
                (m_state == RECOVERING && m_preRecoveryState == LAUNCHING)
                ) {
            this->onLaunchUavComplete();
        } else {
            std::cout << "NS3_TruckDeliveryStrategy::receiveMsg ignored msg delivering" << std::endl;
        }
    }

    //TODO: Add timeout for ACK and NACK reception

    if (_msg.find("NACK_") != std::string::npos) {
        std::string originalMsg = _msg;
        originalMsg.erase(0,5);
        Simulator::Schedule(Seconds(2), &NS3_TruckDeliveryStrategy::sendMsg, this, originalMsg, _senderIndex);
    }
}

void NS3_TruckDeliveryStrategy::sendMsg(std::string _msg, uint _uavIndex)
{
    if (m_communicationsEnabled) {
        m_transciever->send(_msg, _uavIndex);
    } else {
        m_UavDeliveryBehaviors.at(_uavIndex)->receiveDirectMsg(_msg);
    }
}

int NS3_TruckDeliveryStrategy::getUnloadingDelay()
{
    return std::get<int>(*delivery::DeliverySettingsService::getInstance()->getDeliverySetting(
                delivery::DeliverySettingsService::DeliverySettingKey::TRUCK_UNLOADING_DELAY
                ));
}

} // namespace StrategicModels
} // namespace NS3
} // namespace LIMoSim
