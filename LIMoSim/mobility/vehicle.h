#ifndef LIMOSIM_VEHICLE_H
#define LIMOSIM_VEHICLE_H

#include "mobilitymodel.h"
#include "LIMoSim/simulation/eventhandler.h"
#include "LIMoSim/world/vector3d.h"
#include "LIMoSim/world/orientation3d.h"
#include "LIMoSim/simulation/mobilitydataexporter.h"
#include "LIMoSim/world/localvehiclemanager.h"

namespace LIMoSim
{

class Vehicle : public EventHandler
{
public:
    Vehicle(const std::string &_id, const std::string &_type);
    virtual ~Vehicle();

    //
    void setAcceleration(const Vector3d &_acceleration);
    void setPosition(const Vector3d &_position);
    void setOrientation(const Orientation3d &_orientation);
    void setVelocity(const Vector3d &_velocity);
    void setOrientationVelocity(const Orientation3d &_orientationVelocity);
    std::string getId();
    std::string getType();
    Vector3d getAcceleration();
    Vector3d getPosition();
    Orientation3d getOrientation();
    Vector3d getVelocity();
    Orientation3d getOrientationVelocity();
    MobilityModel* getModel();

    //
    LocalVehicleManager *getLocalVehicleManager();


protected:
    void initialize();
    void handleEvent(Event *_event);
    void handleMoveEvent(Event *_event);
    virtual void move(double _timeDelta_s) = 0;

    void setModel(MobilityModel *_mobilityModel);

protected:
    std::string m_id;
    std::string m_type;

    Vector3d m_position; // x,y,z
    Orientation3d m_orientation; // pitch,roll,yaw
    Vector3d m_velocity_mps;
    Orientation3d m_orientationVelocity_radps = Orientation3d(0,0,0);
    Vector3d m_acceleration_mps2;

    Event *m_updateTimer;
    double m_updateInterval_s;
    double m_lastUpdate_s;

    MobilityDataExporter *m_mobilityDataExporter;
    MobilityModel* m_mobilityModel;

    double moveSpeedUp;
};

}

#endif // LIMOSIM_VEHICLE_H
