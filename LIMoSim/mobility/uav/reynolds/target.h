#ifndef TARGET_H
#define TARGET_H

#include "LIMoSim/world/vector3d.h"

namespace LIMoSim {

class UAV;

class Target
{
public:
    Target(Vector3d _point);
    Target(std::string _vehicleId);
    Target(Target const & _ref);

    bool isMobile();

    Vector3d getPosition();
    Vector3d getVelocity();

    std::string getVehicleId();

    void set(Vector3d _point);
    void set(std::string _vehicleId);

private:
    Vector3d m_point;
    std::string m_vehicleId;
    bool m_isMobile;
};

} // namespace LIMoSim

#endif // TARGET_H
