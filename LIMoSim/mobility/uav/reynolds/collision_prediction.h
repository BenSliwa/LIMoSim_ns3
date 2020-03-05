#ifndef COLLISION_PREDICTION_H
#define COLLISION_PREDICTION_H

#include "LIMoSim/world/vector3d.h"
#include "LIMoSim/mobility/mobilitydata.h"

namespace LIMoSim {

class Vehicle;

struct CollisionPrediction {
    double timeFromPresent;
    Vector3d position;
    MobilityData vehicleMobilityData;

    CollisionPrediction(double _timeFromPresent, Vector3d _position, MobilityData _vehicleMobilityData):
        timeFromPresent(_timeFromPresent),
        position(_position),
        vehicleMobilityData(_vehicleMobilityData)
    {}
};

}
#endif // COLLISION_PREDICTION_H
