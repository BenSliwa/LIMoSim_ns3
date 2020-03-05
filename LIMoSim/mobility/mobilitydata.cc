#include "mobilitydata.h"

namespace LIMoSim {

MobilityData::MobilityData(std::string _name):
    name(_name)
{
    if (name.empty()) {
        state = INVALID;
    }
}

MobilityData::MobilityData(Vector3d _position,
                           Orientation3d _orientation,
                           Vector3d _velocity,
                           Orientation3d _orientationVelocity,
                           Vector3d _acceleration,
                           std::string _name):
    acceleration(_acceleration),
    position(_position),
    orientation(_orientation),
    velocity(_velocity),
    orientationVelocity(_orientationVelocity),
    name(_name),
    state(VALID)
{

}

std::ostream &MobilityData::serialize(std::ostream &os) const
{
    os << position << " " << orientation << " " <<velocity << " " <<acceleration << " " << name << " \0";
    return os;
}

std::istream &MobilityData::deserialize(std::istream &is)
{
    is >> position >> orientation >> velocity >> acceleration >> name;
    if (name.empty()) {
        state = INVALID;
    } else {
        state = VALID;
    }
    return is;
}

} // namespace LIMoSim
