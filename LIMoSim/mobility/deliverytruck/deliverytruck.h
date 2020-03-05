#ifndef DELIVERYTRUCK_H
#define DELIVERYTRUCK_H

#include "LIMoSim/mobility/car/car.h"

namespace LIMoSim {

class TruckDelivery;

namespace mobility {
namespace car {

class DeliveryTruck: public Car
{
public:
    DeliveryTruck(
            const std::string &_id,
            const uint _delivererCount,
            std::vector<std::string> _deliveryList);
    ~DeliveryTruck();

    const std::vector<std::string>& getDeliveryList();
    uint getDelivererCount() const;
    Vector3d getCurrentDeliveryTargetPosition();

    bool isStopped();


    // Event Handler Interface
    void initialize();
    void handleEvent(Event *_event);

    // Vehicle Interface
    void move(double _timeDelta_s);
    void updateIntersectionAwareness();

    void setStrategy(TruckDelivery *_strategy);
    bool isAnyUavNearby(double _radius = 10);

protected:
    uint m_delivererCount;
    std::vector<std::string> m_deliveryList;
    TruckDelivery *p_truckDeliveryStrategy;

    bool m_stopped;

};

} // namespace var
} // namespace m,bility
} // namespace LIMoSim

#endif // DELIVERYTRUCK_H
