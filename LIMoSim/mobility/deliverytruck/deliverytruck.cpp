#include "deliverytruck.h"

#include "LIMoSim/mobility/car/following/idm.h"
#include "LIMoSim/mobility/car/strategic/truckdelivery.h"
#include "LIMoSim/world/world.h"
#include "LIMoSim/world/vehiclemanager.h"
#include "demo/deliverylistservice.h"

namespace LIMoSim {
namespace mobility {
namespace car {

DeliveryTruck::DeliveryTruck(
        const std::string &_id,
        const uint _delivererCount,
        std::vector<std::string> _deliveryList
        ):
    Car(_id),
    m_delivererCount(_delivererCount),
    m_deliveryList(_deliveryList),
    m_stopped(false)
{
    delete m_strategic;
    p_truckDeliveryStrategy = new TruckDelivery(this);
    m_strategic = p_truckDeliveryStrategy;
}

DeliveryTruck::~DeliveryTruck()
{
    // TODO: Delete strategy
}

const std::vector<std::string> &DeliveryTruck::getDeliveryList()
{
    return m_deliveryList;
}

uint DeliveryTruck::getDelivererCount() const
{
    return m_delivererCount;
}

Vector3d DeliveryTruck::getCurrentDeliveryTargetPosition()
{
    Endpoint *endpoint = p_truckDeliveryStrategy->getNextEndpoint();
    if (endpoint && p_truckDeliveryStrategy->getState() == DELIVERING) {
        return endpoint->getNode()->getPosition();
    } else {
        return p_truckDeliveryStrategy->getcurrentWaypoint();
    }
}

bool DeliveryTruck::isStopped()
{
    return m_stopped;
}

void DeliveryTruck::initialize()
{
    m_strategic->initialize();
    Vehicle::initialize();

    m_speed_mps = 0;
}

void DeliveryTruck::handleEvent(Event *_event)
{
    if (_event->getInfo() == "TruckDelivery::Stop") {
        std::cout << "DeliveryTruck::HandleEvent stopping " << std::endl;
        m_stopped = true;
        delete _event;
    } else if (_event->getInfo() == "TruckDelivery::Start") {
        std::cout << "DeliveryTruck::HandleEvent starting " << std::endl;
        m_stopped = false;
        delete _event;
    }else {
        Vehicle::handleEvent(_event);
    }
}

void DeliveryTruck::move(double _timeDelta_s)
{
    p_truckDeliveryStrategy->checkEnRouteDroneHandling();

    // update the road awareness
    RoadAwarenessEntry leader = m_awareness.findLeader(this);
    if(leader.car)
    {
        std::cout << "leader for " << m_id << ":\t" << leader.car->getId() << "\td: " << leader.distance_m << " m" << std::endl;
    }

    if (m_stopped) {
        std::cout << "DeliveryTruck::Move skipped because vehicle is stopped" << std::endl;
        return;
    }

    // update acceleration and speed
    double allowedSpeed_mps = m_roadPosition.laneSegment->getSegment()->getRoad()->getMaxSpeed();
    if (isAnyUavNearby(80) && /*!Demo::DeliveryListService::getInstance()->getPendingCount() &&*/
            !delivery::DeliveryListService::getInstance()->allDroneTargetsDelivered()) {
        allowedSpeed_mps =5;
    }

    auto truckStrategy = dynamic_cast<TruckDelivery*>(m_strategic);
    if (truckStrategy) {
        // Pretend delivery endpoints are intersections to force
        // some deceleration upon approaching them
        double endpointDist = truckStrategy->distanceToNextEndpoint();
        if (endpointDist/2 < m_roadPosition.distanceToIntersection_m) {
            m_roadPosition.distanceToIntersection_m = endpointDist;
        }
    }
    m_acceleration_mpss = 0; //TODO: m_following->computeAcceleration(m_speed_mps, allowedSpeed_mps, leader);
    m_speed_mps += m_acceleration_mpss;
    m_velocity_mps = m_roadPosition.laneSegment->computeDirection().normed() * m_speed_mps;

    // compute the travelled distance
    double distance_m = m_speed_mps * _timeDelta_s;
    double remainingDistance_m = m_roadPosition.laneSegment->computeLength() - m_roadPosition.offset_m;


    //
    if(distance_m < remainingDistance_m)
    {
        m_roadPosition.offset_m += distance_m;
        setRoadPosition(m_roadPosition);
    }
    else // switch lane segment
    {

        LaneSegment *currentSegment = m_roadPosition.laneSegment;
        LaneSegment *nextSegment = truckStrategy->determineNextLaneSegment(currentSegment->getEndNode());
        Node *node = currentSegment->getEndNode();
        Intersection* intersection = p_world->getIntersection(node);
        Gate *gate = 0;
        if(intersection)
            gate = intersection->getInGate(currentSegment->getSegment());

        // call the node update handler for statistics etc.
        m_strategic->handleNodeReached(node);


        //
        if(gate)
        {
            truckStrategy->handleGateReached(gate, intersection, currentSegment, nextSegment);
        }
        else if(nextSegment)
        {
            m_roadPosition.laneSegment = nextSegment;
            m_roadPosition.offset_m = 0;
            setRoadPosition(m_roadPosition);
        }
    }

    //
    updateIntersectionAwareness();
}

void DeliveryTruck::updateIntersectionAwareness()
{
    LaneSegment *ls = m_roadPosition.laneSegment;
    m_roadPosition.distanceToIntersection_m = ls->computeLength() - m_roadPosition.offset_m;
    Intersection *intersection = nullptr;
    while(!intersection)
    {
        // intersection detecion
        intersection = p_world->getIntersection(ls->getEndNode());
        if(!intersection)
        {
            ls = ls->getNext();
            if(!ls)
                std::cout << "INVALID LANE - Car::updateIntersectionAwareness" << std::endl;
            else
                m_roadPosition.distanceToIntersection_m += ls->computeLength();
        }
    }
}

void DeliveryTruck::setStrategy(TruckDelivery *_strategy)
{
    TruckDelivery * previous = p_truckDeliveryStrategy;
    p_truckDeliveryStrategy = _strategy;
    m_strategic = _strategy;
    delete previous;
}

bool DeliveryTruck::isAnyUavNearby(double _radius)
{
    MobilityDataMap dataMap = getLocalVehicleManager()->getMobilityData();

    for (auto entry : dataMap) {
        MobilityData data = entry.second;
        if((data.position - getPosition()).norm() <= _radius) {
            return true;
        }
    }
    return false;
}

} // namespace car
} // namespace mobility
} // namespace LIMoSim
