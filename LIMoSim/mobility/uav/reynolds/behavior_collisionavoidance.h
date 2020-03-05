#ifndef BEHAVIOR_COLLISIONAVOIDANCE_H
#define BEHAVIOR_COLLISIONAVOIDANCE_H

#include <map>
#include <vector>
#include "LIMoSim/mobility/uav/reynolds/behavior.h"
#include "collision_prediction.h"

namespace LIMoSim {

class UAV;
class Vehicle;
class MobilityData;
class Behavior_CollisionAvoidance : public Behavior
{
public:
    Behavior_CollisionAvoidance(double _collisionSearchRadius= 50, double _collisionSphereRadius = 5.0, std::string _agentId="");

    // Action interface
    /**
     * @brief apply
     * The collision avoidances returns a steering for collision avoidance
     * only if the agent is moving. If not then the returned corrective steering
     * shall have no effect;
     * @return
     */
    Steering apply();

    /**
     * @brief composeWith
     * Wrap a specified behavior in a behavior composition with
     * collision avoidance in priority by order mode.
     * @param _behavior
     * @param _collisionSearchRadius
     * @param _collisionSphereRadius
     * @return
     */
    static Behavior* composeWith(Behavior* _behavior, double _collisionSearchRadius = 50.0, double _collisionSphereRadius = 5.0);
    /**
     * @brief composeWith
     * Wrap a specified behavior in a behavior composition with
     * collision avoidance in priority by order mode.
     * @param _behavior
     * @param _collisionSearchRadius
     * @param _collisionSphereRadius
     * @return
     */
    static Behavior* composeWith(Behavior* _behavior, std::string _agentId, double _collisionSearchRadius = 50.0, double _collisionSphereRadius = 5.0);

protected:
    /**
     * @brief cpaTime
     * computes the time at which the agent and Vehicle will be the nearest
     * @param v
     * @return the time of the closest approach with v
     */
    double cpaTime(MobilityData v);

    /**
     * @brief cpaDistance
     * @param v
     * @return the distance between the agent and vehicle v at their
     *  Closest Point of Approach
     */
    double cpaDistance(MobilityData v);
    std::map<std::string, MobilityData> findNearByVechicles();
    std::vector<CollisionPrediction> findFutureCollisions();
    /**
     * @brief predictCollision
     * Predicts eventual collision with vehicle v if one may or will have happened
     * in the agent's local time frame.
     * @param v
     * @return
     */
    CollisionPrediction predictCollision(MobilityData v);
    /**
     * @brief vehicleIsLeft
     * @param v
     * @return true if the agent's position vector is left of agent v's position vector
     */
    bool vehicleIsLeft(MobilityData v);
    double map(double x1, double x2, double y1, double y2, double value);

    static double s_parallelVelocitiesThreshold;

private:
    double m_collisionSphereRadius;
    double m_searchRadius;
};

} // namespace LIMoSim

#endif // BEHAVIOR_COLLISIONAVOIDANCE_H
