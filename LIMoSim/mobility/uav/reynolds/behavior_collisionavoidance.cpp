#include "behavior_collisionavoidance.h"
#include "LIMoSim/mobility/uav/reynolds/behavior_composition.h"
#include "LIMoSim/world/vehiclemanager.h"
#include <algorithm>

namespace LIMoSim {

double Behavior_CollisionAvoidance::s_parallelVelocitiesThreshold = 1e-8;

Behavior_CollisionAvoidance::Behavior_CollisionAvoidance(double _searchRadius, double _collisionSphereRadius, std::string _agentId):
    Behavior("CollisionAvoidance", _agentId),
    m_collisionSphereRadius(_collisionSphereRadius),
    m_searchRadius(_searchRadius)
{

}

Steering Behavior_CollisionAvoidance::apply()
{
    std::vector<CollisionPrediction> collisions = findFutureCollisions();

    if (collisions.size() > 0) {

        CollisionPrediction nextCollision = collisions[0];

        double collisionDistance = (nextCollision.position - getAgent()->getPosition()).norm();
        double brakeFactor =  map(m_collisionSphereRadius * 2, m_searchRadius, 1, 0.0, collisionDistance);

        Vector3d vel = getAgent()->getVelocity();

        double angle = Vector3d::angle(getAgent()->getVelocity(), nextCollision.vehicleMobilityData.velocity);

        if (angle < 45) {
            bool faster = getAgent()->getVelocity().norm() > nextCollision.vehicleMobilityData.velocity.norm();
            return Steering(Orientation3d(0), faster ? vel.rotateRight() : vel.rotateLeft());
        } else {

            if (vehicleIsLeft(nextCollision.vehicleMobilityData)) {
                if (vel.norm() < getAgent()->getModel()->getLocomotion()->getVelocityMax()) {
                    // speed up and steer right
                    return Steering(Orientation3d(0), (vel * brakeFactor +  vel.rotateRight()));
                } else {
                    // slow down and steer left
                    return Steering(Orientation3d(0), (vel.rotateLeft() - vel * brakeFactor));
                }
            } else {
                // slow down and steer right
                return Steering(Orientation3d(0), (vel.rotateRight() - vel * brakeFactor));
            }
        }

    }
    return Steering();
}

Behavior *Behavior_CollisionAvoidance::composeWith(Behavior *_behavior, double _collisionSearchRadius, double _collisionSphereRadius)
{
    return new Behavior_Composition(
                _behavior->getName() + " with Collision Avoidance",
                Behaviors {
                    new Behavior_CollisionAvoidance(_collisionSearchRadius, _collisionSphereRadius),
                    _behavior
                }
                );
}

Behavior *Behavior_CollisionAvoidance::composeWith(Behavior *_behavior, std::string _agentId, double _collisionSearchRadius, double _collisionSphereRadius)
{
    Behavior *b = Behavior_CollisionAvoidance::composeWith(_behavior, _collisionSearchRadius, _collisionSphereRadius);
    b->setAgent(_agentId);
    return b;
}

double Behavior_CollisionAvoidance::cpaTime(MobilityData v)
{
    Vector3d dv = getAgent()->getVelocity() - v.velocity;

    double dv2 = Vector3d::dot(dv,dv);

    if (dv2 < s_parallelVelocitiesThreshold) { // tracks are almost parallel
        return 0.0;
    }

    Vector3d w0 = getAgent()->getPosition() - v.position;
    double time = -Vector3d::dot(w0,dv)/dv2;

    return time;
}

double Behavior_CollisionAvoidance::cpaDistance(MobilityData v)
{
    double _cpaTime = cpaTime(v);
    Vector3d p1Atcpa = getAgent()->getPosition() + getAgent()->getVelocity() * _cpaTime;
    Vector3d p2Atcpa = v.position + v.velocity * _cpaTime;
    return (p2Atcpa - p1Atcpa).norm();
}

std::map<std::string, MobilityData> Behavior_CollisionAvoidance::findNearByVechicles()
{

    auto mobilityDataReport = getAgent()->getLocalVehicleManager()->getMobilityData();

    std::map<std::string, MobilityData> _nearby;

    for (auto it : mobilityDataReport) {
        MobilityData data = it.second;
        if ( (data.position - getAgent()->getPosition()).norm() <= m_searchRadius ){
            _nearby.insert((_nearby.end(), it));
        }
    }

    return _nearby;

//    auto uavs = VehicleManager::getInstance()->getVehicles();
//    std::map<std::string, Vehicle*> nearby;

//    for (auto it = uavs.begin(); it != uavs.end(); it++) {
//        Vehicle * v = it->second;
//        if ( v != getAgent() && (v->getPosition() - getAgent()->getPosition()).norm() <= m_searchRadius) {
//            nearby.insert(nearby.end(), (*it));
//        }
//    }
//    return nearby;
}

std::vector<CollisionPrediction> Behavior_CollisionAvoidance::findFutureCollisions()
{
    std::vector<CollisionPrediction> collisions;


    MobilityDataMap nearby = findNearByVechicles();


//    VehicleMap nearby = findNearByVechicles();

    for (auto iterator : nearby) {
        CollisionPrediction prediction = predictCollision(iterator.second);
        if (prediction.vehicleMobilityData.getState()) { collisions.push_back(prediction);}
    }

    std::sort(
                collisions.begin(),
                collisions.end(),
                [](CollisionPrediction left, CollisionPrediction right) -> bool {
                    return left.timeFromPresent < right.timeFromPresent;
                }
                );

    return collisions;
}

CollisionPrediction Behavior_CollisionAvoidance::predictCollision(MobilityData v)
{
    /*
     *From:
     * - http://geomalgorithms.com/a07-_distance.html
     */
    double _cpaTime = cpaTime(v);

    if (!_cpaTime) { //nearly parallel velocities
        double currentDistance = (getAgent()->getPosition() - v.position).norm();
        if (currentDistance < 2 * m_collisionSphereRadius) {
            return CollisionPrediction(_cpaTime, getAgent()->getPosition(), v);
        }
    }

    double _cpaDistance = cpaDistance(v);

    if (_cpaDistance < (2 * m_collisionSphereRadius)) { // distance at closest approach smaller than collision spheres radii
        return CollisionPrediction(_cpaTime, getAgent()->getPosition() + getAgent()->getVelocity() * _cpaTime, v);
    }

    // no threatening collision prediction
    return CollisionPrediction(0,Vector3d(), MobilityData());
}

bool Behavior_CollisionAvoidance::vehicleIsLeft(MobilityData v)
{
    Vector3d p1 = getAgent()->getPosition();
    Vector3d p2 = v.position;

    double sinus = (p1.x*p2.y - p1.y*p2.x) /(p1.norm() * p2.norm());

    return sinus > 0;
}

double Behavior_CollisionAvoidance::map(double x1, double x2, double y1, double y2, double value)
{
    double mapped = (value - x1) * (y2-y1)/(x2-x1) + y1;

    mapped = std::min(std::max(y1,y2), mapped);
    mapped = std::max(std::min(y1, y2), mapped);
    return mapped;
}


} // namespace LIMoSim
