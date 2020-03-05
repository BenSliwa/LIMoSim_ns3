#ifndef MOBILITYMODEL_H
#define MOBILITYMODEL_H

#include "LIMoSim/mobility/uav/reynolds/locomotionupdate.h"

namespace LIMoSim {

class Vehicle;

class MobilityModel
{
public:
    MobilityModel(std::string _agentId);

    Vehicle* getAgent();

    virtual LocomotionUpdate step(double _timeDelta_s) = 0;
protected:
    std::string m_agentId;
};

} // namespace LIMoSim

#endif // MOBILITYMODEL_H
