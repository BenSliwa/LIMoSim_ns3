#include "vehiclemanager.h"

#include "energy/energymonitor.h"

namespace LIMoSim
{

VehicleManager::VehicleManager():
    m_broadcastMobilityInterval_s(0.5),
    m_lastBroadcast_s(0),
    m_deterministicCarPath(false)
{

}

VehicleManager::~VehicleManager()
{

    // delete local vehicle managers
    for (
         auto it = m_localVehicleManagers.begin();
         it != m_localVehicleManagers.end();
         it++) {
        m_localVehicleManagers.erase(it);
        delete (it->second);
    }
}

VehicleManager* VehicleManager::getInstance()
{
    static VehicleManager instance;
    return &instance;
}


/*************************************
 *            PUBLIC METHODS         *
 ************************************/

Car* VehicleManager::createCar(const std::string &_id)
{
    Car *car = new Car(_id);
    m_vehicles[_id] = car;

    registerLocalVehicleManager(car->getId(), new LocalVehicleManager(car));
    setUIDataForVehicle(car);

    return car;
}

DeliveryTruck *VehicleManager::createDeliveryTruck(
        const std::string &_id,
        const uint _delivererCount,
        std::vector<std::string> &_deliveryList
        )
{
    DeliveryTruck *truck = new DeliveryTruck(
                _id,
                _delivererCount,
                _deliveryList
                );
    m_vehicles[_id] = truck;

    registerLocalVehicleManager(truck->getId(), new LocalVehicleManager(truck));
    setUIDataForVehicle(truck);

    return truck;
}

UAV* VehicleManager::createUAV(const std::string &_id)
{
    UAV *uav = new UAV(_id);
    m_vehicles[_id] = uav;

    registerLocalVehicleManager(uav->getId(), new LocalVehicleManager(uav));
    setUIDataForVehicle(uav);
    Energy::EnergyMonitor::getInstance()->startMonitoring(_id);

    return uav;
}

void VehicleManager::clearVehicles()
{
    for (auto it = m_vehicles.begin(); it != m_vehicles.end(); it++) {
        deleteVehicle(it->first);
    }
    m_vehicles.clear();
    initialize();
}

void VehicleManager::deleteVehicle(const std::string &_id)
{
    if(m_vehicles.count(_id)>0)
    {
        Vehicle *vehicle = m_vehicles[_id];
        Energy::EnergyMonitor::getInstance()->stopMonitoring(_id);
        unregisterLocalVehicleManager(_id);
        deleteUIDataForVehicle(_id);
        delete vehicle;
    }
}

bool VehicleManager::hasVehicle(const std::string &_id)
{
    if(m_vehicles.count(_id)>0)
        return true;
    return false;
}

Vehicle* VehicleManager::getVehicle(const std::string &_id)
{
    if(m_vehicles.count(_id)>0)
        return m_vehicles[_id];
    return 0;
}

const std::map<std::string, Vehicle*>& VehicleManager::getVehicles()
{
    return m_vehicles;
}

std::map<std::string, Vehicle *> VehicleManager::getVehicles(const std::string &_type)
{
    std::map<std::string, Vehicle*> vehicles;
    for (auto it = m_vehicles.begin(); it != m_vehicles.end(); it++) {
        if (it->second->getType() == _type) {
            vehicles.insert(vehicles.end(),*(it));
        }
    }
    return vehicles;
}

bool VehicleManager::getDeterministicCarPath()
{
    return m_deterministicCarPath;
}

void VehicleManager::setDeterministicCarPathEnabled(bool _enabled)
{
    m_deterministicCarPath = _enabled;
}

void VehicleManager::registerLocalVehicleManager(std::string _vehicleId, LocalVehicleManager *_localVehicleManager)
{
    m_localVehicleManagers[_vehicleId] = _localVehicleManager;
}

void VehicleManager::unregisterLocalVehicleManager(std::string _vehicleId)
{
    if (m_localVehicleManagers.count(_vehicleId)) {
        delete m_localVehicleManagers[_vehicleId];
        m_localVehicleManagers.erase(_vehicleId);
    }
}

void VehicleManager::broadcastMobilityDataToLocalVehicleManagers()
{
    for (auto entry : m_localVehicleManagers) {
        LocalVehicleManager* localVehicleManager= entry.second;
        localVehicleManager->updateMobilityDataReport(m_mobilityDataReport);
    }
}

LocalVehicleManager *VehicleManager::getLocalVehicleManager(std::string _vehicleId)
{
    if (m_localVehicleManagers.count(_vehicleId)) {
        return m_localVehicleManagers.at(_vehicleId);
    } else {
        return nullptr;
    }
}

void VehicleManager::setUIDataHandler(UIData *_uiData)
{
    p_uiData = _uiData;
}

void VehicleManager::enableMobilityBroadcastHelper()
{
    if (Simulation::getInstance()->running()) {
        std::cerr << "vehicleManager: Mobility broadcast helper must be activated before simulation start!" << std::endl;
        exit(1);
    } else {
        initialize();
    }
}

void VehicleManager::updateMobilityReport(std::string _vehicleId, MobilityData _mobilityData)
{
    m_mobilityDataReport[_vehicleId] = _mobilityData;
}

void VehicleManager::initialize()
{
    p_broadcastMobilityDataTimer = new Event(m_broadcastMobilityInterval_s, this, "BroadcastMobilityData");
    scheduleEvent(p_broadcastMobilityDataTimer);
}

void VehicleManager::handleEvent(Event *_event)
{
    if (_event->getInfo() == p_broadcastMobilityDataTimer->getInfo()) {
        broadcastMobilityDataToLocalVehicleManagers();
    }

    m_lastBroadcast_s = _event->getTimestamp();
    scheduleEvent(_event, m_broadcastMobilityInterval_s);
}

void VehicleManager::setUIDataForVehicle(Vehicle *_vehicle)
{
    if (p_uiData) {
        p_uiData->registerVehicle(_vehicle);
    } /*else {
        std::cerr <<"Vehicle Manager has no UIDataHandler for registering vehicles" << std::endl;
    }*/
}

void VehicleManager::deleteUIDataForVehicle(std::string _vehicleId)
{
    if (p_uiData) {
        p_uiData->unregisterVehicle(_vehicleId);
    }
}


}
