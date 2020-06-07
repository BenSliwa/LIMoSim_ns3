#ifndef LIMOSIM_VEHICLEMANAGER_H
#define LIMOSIM_VEHICLEMANAGER_H

#include "LIMoSim/mobility/car/car.h"
#include "LIMoSim/mobility/uav/uav.h"
#include "LIMoSim/mobility/mobilitydata.h"
#include "LIMoSim/world/localvehiclemanager.h"
#include "LIMoSim/world/uidata.h"
#include "LIMoSim/simulation/eventhandler.h"
#include <map>

namespace LIMoSim
{
typedef std::map<std::string, Vehicle*> VehicleMap;
typedef std::map<std::string, LocalVehicleManager*> LocalVehicleManagerMap;

class VehicleManager : public EventHandler
{
public:
    VehicleManager();
    ~VehicleManager();

    static VehicleManager* getInstance();

    // creation
    Car* createCar(const std::string &_id);
    UAV* createUAV(const std::string &_id);

    // deletion
    void clearVehicles();
    void deleteVehicle(const std::string &_id);
    bool hasVehicle(const std::string &_id);

    //
    Vehicle* getVehicle(const std::string &_id);
    const std::map<std::string, Vehicle*>& getVehicles();
    std::map<std::string, Vehicle*> getVehicles(const std::string & _type);

    bool getDeterministicCarPath();
    void setDeterministicCarPathEnabled(bool _enabled);

    //
    void registerLocalVehicleManager(std::string _vehicleId, LocalVehicleManager *_localVehicleManager);
    void unregisterLocalVehicleManager(std::string _vehicleId);
    void broadcastMobilityDataToLocalVehicleManagers();
    LocalVehicleManager* getLocalVehicleManager(std::string _vehicleId);

    void setUIDataHandler(UIData * _uiData);
    void enableMobilityBroadcastHelper();
    void updateMobilityReport(std::string _vehicleId, MobilityData _mobilityData);

protected:
    void initialize();
    void handleEvent(Event *_event);

private:
    std::map<std::string, Vehicle*> m_vehicles;
    LocalVehicleManagerMap m_localVehicleManagers;
    MobilityDataMap m_mobilityDataReport;

    bool m_deterministicCarPath;

    /**
     * @brief p_uiData
     * Singleton => not managed by this class.
     * Do not delete.
     */
    UIData* p_uiData;

    Event *p_broadcastMobilityDataTimer;
    double m_broadcastMobilityInterval_s;
    double m_lastBroadcast_s;

    void setUIDataForVehicle(Vehicle *_vehicle);
    void deleteUIDataForVehicle(std::string _vehicleId);
};


}

#endif // LIMOSIM_VEHICLEMANAGER_H
