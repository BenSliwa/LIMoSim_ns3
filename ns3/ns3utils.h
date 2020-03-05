#ifndef NS3UTILS_H
#define NS3UTILS_H


// LIMoSim
#include "LIMoSim/world/vector3d.h"

// ns3
#include "ns3/vector.h"
#include <ns3/callback.h>

namespace LIMoSim {

enum VehicleType { LIMoSimCar=0, LIMoSimUAV };
class Behavior;

namespace NS3 {

using namespace ns3;


extern bool logAnimationWarnings;

/**
 * @brief toObstacleShadowingCacheKey
 * Transform position pair in usable key for
 * obstacle shadowing cache
 * @param _a
 * @param _b
 * @return the corresponding obstacle shadowing cache key
 */
inline std::string toObstacleShadowingCacheKey(const Vector & _a, const Vector & _b) {
    std::stringstream stream;
    stream << static_cast<int>(_a.x)
           << static_cast<int>(_a.y)
           << static_cast<int>(_a.z)
           << static_cast<int>(_b.x)
           << static_cast<int>(_b.y)
           << static_cast<int>(_b.z);
    return stream.str();
}


Vector3d toLIMoSimVector(Vector _ns3Vector);
Vector toNS3Vector(Vector3d _LIMoSimVector);

struct VehicleUIProps {
    std::string color;
    VehicleUIProps(std::string _color="white");
};

struct VehicleProperties {
    Vector position;
    VehicleType type;
    std::string id;
    VehicleUIProps ui;
    Callback<Behavior*> behaviorFactory;
    VehicleProperties(Vector _position,
            VehicleType _type,
            std::string _id
            );
    VehicleProperties(Vector _position,
            VehicleType _type,
            std::string _id,
            VehicleUIProps _ui,
            Callback<Behavior*> _behaviorFactory = MakeNullCallback<Behavior*>()
            );
};
} // namespace NS3

namespace UI {
namespace AnimationUtils {

void animateReception(std::string _nodeId);
void animateTransmission(std::string _nodeId);

} // namespace AnimationUtils
} // namespace UI
} // namespace LIMoSim

#endif // NS3UTILS_H
