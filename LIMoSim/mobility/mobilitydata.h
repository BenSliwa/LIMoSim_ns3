#ifndef MOBILITYDATA_H
#define MOBILITYDATA_H

#include <string>
#include "LIMoSim/world/vector3d.h"
#include "LIMoSim/world/orientation3d.h"

namespace LIMoSim {

class MobilityData
{
public:
    MobilityData(std::string _name = "");
    MobilityData(Vector3d _position,
                 Orientation3d _orientation,
                 Vector3d _velocity,
                 Orientation3d _orientationVelocity,
                 Vector3d _acceleration,
                 std::string _name);

    Vector3d acceleration;
    Vector3d position;
    Orientation3d orientation;
    Orientation3d orientationVelocity;
    Vector3d velocity;
    std::string name;

    enum State { INVALID = 0, VALID};

    State getState() { return state; }

    std::ostream& serialize(std::ostream& os) const;
    std::istream& deserialize(std::istream& is);

private:
    State state;

};

} // namespace LIMoSim

#endif // MOBILITYDATA_H
