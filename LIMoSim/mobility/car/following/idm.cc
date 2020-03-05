#include "idm.h"



namespace LIMoSim
{

IDM::IDM(Car *_car) :
    FollowingModel(_car),
    m_safeTimeHeadway_s(1.5),
    m_maxAcceleration_mpss(1),
    m_maxDeceleration_mpss(3), // high impact on the min distance (safety gap before busy road is considered)
    m_accelerationExponent(4.0),
    m_minDistance_m(2.0)
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

double IDM::computeAcceleration(double _speed_mps, double _desiredSpeed_mps, bool _hasLeader, double _speedDelta_mps, double _distance_m)
{
    double freeRoadAcceleration = computeFreeRoadCoefficient(_speed_mps, _desiredSpeed_mps);
    double busyRoadAcceleration = 0;
    if(_hasLeader && _distance_m>0)
        busyRoadAcceleration = computeBusyRoadCoefficient(_speed_mps, _speedDelta_mps, _distance_m);

    double acceleration_mpss = m_maxAcceleration_mpss * (1 - freeRoadAcceleration - busyRoadAcceleration);

    return limitAcceleration(acceleration_mpss);
}


/*************************************
 *           PRIVATE METHODS         *
 ************************************/

double IDM::computeFreeRoadCoefficient(double _speed_mps, double _desiredSpeed_mps)
{
    double coefficient = pow((_speed_mps/_desiredSpeed_mps), m_accelerationExponent);
    if(_desiredSpeed_mps==0)
        coefficient = 1;

    return coefficient;
}

double IDM::computeBusyRoadCoefficient(double _speed_mps, double _speedDelta_mps, double _distance_m)
{
    double desiredGap_m = computeDesiredGap(_speed_mps, _speedDelta_mps);
    double coefficient = pow((desiredGap_m/_distance_m), 2);

    return coefficient;
}

double IDM::computeDesiredGap(double _speed_mps, double _speedDelta_mps)
{
    // caution: s0 + s1 in reality
    double breakGap_m = computeBreakGap(_speed_mps, _speedDelta_mps);
    double timeGap_m = _speed_mps * m_safeTimeHeadway_s;
    double distance_m = m_minDistance_m + timeGap_m + breakGap_m;

    return distance_m;
}

double IDM::computeBreakGap(double _speed_mps, double _speedDelta_mps)
{
    double distance_m = _speed_mps * _speedDelta_mps / (2 * sqrt(m_maxAcceleration_mpss * m_maxDeceleration_mpss));

    return distance_m;
}

double IDM::limitAcceleration(double _acceleration)
{
    double acceleration_mpss = _acceleration;
    if(acceleration_mpss>m_maxAcceleration_mpss)
        acceleration_mpss = m_maxAcceleration_mpss;
    else if(acceleration_mpss<-m_maxAcceleration_mpss)
        acceleration_mpss = -m_maxAcceleration_mpss;

    return acceleration_mpss;
}

}
