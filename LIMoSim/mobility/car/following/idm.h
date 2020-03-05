#ifndef LIMOSIM_IDM_H
#define LIMOSIM_IDM_H

#include "followingmodel.h"
#include "LIMoSim/mobility/car/awareness/roadawareness.h"
#include "LIMoSim/world/road/intersection.h"

namespace LIMoSim
{


class IDM : public FollowingModel
{
public:
    IDM(Car *_car);

    double computeAcceleration(double _speed_mps, double _desiredSpeed_mps, bool _hasLeader = false, double _speedDelta_mps = 0, double _distance_m = 0);

private:
    double computeFreeRoadCoefficient(double _speed_mps, double _desiredSpeed_mps);
    double computeBusyRoadCoefficient(double _speed_mps, double _speedDelta_mps, double _distance_m);

    double computeDesiredGap(double _speed_mps, double _speedDelta_mps);
    double computeBreakGap(double _speed_mps, double _speedDelta_mps);

    double limitAcceleration(double _acceleration);

private:
    double m_safeTimeHeadway_s;
    double m_maxAcceleration_mpss;
    double m_maxDeceleration_mpss;
    double m_accelerationExponent;
    double m_minDistance_m;
};

}


#endif // LIMOSIM_IDM_H
