#ifndef UAV_REYNOLDSMODEL_H
#define UAV_REYNOLDSMODEL_H

#include "LIMoSim/mobility/uav/reynolds/reynoldsmodel.h"

#include "uav_locomotion.h"

namespace LIMoSim {

class UAV;
class Vehicle;

class UAV_ReynoldsModel: public ReynoldsModel
{
public:
    UAV_ReynoldsModel(std::string _agentId);
    ~UAV_ReynoldsModel();

    Vector3d getWaypoint();    
    /**
     * @brief getCarryingVehicle
     * @return  the vehicle carrying the one this model belongs to.
     */
    Vehicle *getCarryingAgent();
    /**
     * @brief agentIsCarried
     * @return tells if the agent this model belongs to is currently
     * being carried by another one.
     */
    bool agentIsCarried();
    /**
     * @brief setCarryingVehicle
     * Notifies the model the vehicle it belongs to is being carried.
     * @param _vehicleId the id of the vehicle instance carrying the one this model belongs to.
     */
    void setCarryingAgent(std::string _vehicleId);
    /**
     * @brief unsetCarrying
     * Notifies the model the the vehicle it belongs
     * is not being carried anymore.
     */
    void unsetCarrying();

protected:
    /**
     * @brief m_carried
     * if the uav is being carried.
     * This has consequences for other parts of the model.
     * Mostly the locomotion, as the UAV does not actively moves
     * anymore but its position gets updated nevertheless.
     * NB: This should be activated whenever docking behavior is active.
     */
    bool m_carried;
    /**
     * @brief m_carryingAgent
     * The vehicle currently carrying the one this model belongs to.
     */
    Vehicle* m_carryingAgent;

    // ReynoldsModel interface
public:
    virtual UAV_Locomotion *getLocomotion() override;
};
}

#endif // UAV_REYNOLDSMODEL_H
