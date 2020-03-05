#ifndef LIMOSIMMOBILITYMODEL_H
#define LIMOSIMMOBILITYMODEL_H

#include <ns3/object.h>
#include <ns3/mobility-model.h>
#include <ns3/traced-callback.h>

#include "ns3utils.h"


namespace LIMoSim {

class Vehicle;
class Behavior;

namespace NS3 {

using namespace ns3;

class LimoSimMobilityModel : public ns3::MobilityModel
{
public:
    static TypeId GetTypeId (void);
    std::string GetVehicleId();
    void  SetVehicleId(std::string _vehicleId);
    Vehicle* GetVehicle() const;

private:
    virtual void DoDispose (void);
    virtual void DoInitialize ();
    virtual Vector DoGetPosition (void) const;
    virtual void DoSetPosition (const Vector &position);
    virtual Vector DoGetVelocity (void) const;
    virtual int64_t DoAssignStreams (int64_t);

    std::string m_vehicleId;
    VehicleType m_vehicleType;
    std::string m_shapeColor;

    Callback<Behavior*> m_behaviorFactory;

    Vector m_initialPosition;
};

}

}

#endif // LIMOSIMMOBILITYMODEL_H
