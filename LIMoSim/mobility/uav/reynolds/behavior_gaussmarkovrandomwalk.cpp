#include "behavior_gaussmarkovrandomwalk.h"

#include <random>

#include "LIMoSim/mobility/uav/uav.h"
#include "behavior_selfalignment.h"


namespace LIMoSim {

Behavior_GaussMarkovRandomWalk::Behavior_GaussMarkovRandomWalk(double _independenceLevel, double _gaussRandomNoiseVariance, std::string _agentId):
    Behavior("Gauss-Markov Random Walk", _agentId),
    m_independenceLevel(_independenceLevel),
    m_gaussRandomNoiseVariance(_gaussRandomNoiseVariance),
    m_stateCount(0)
{

    std::random_device rd{};

    // create a standard normal distrbuted random number generator
    // for each dimension
    m_normalDistros.push_back(std::normal_distribution<> {0,1});
    m_normalDistros.push_back(std::normal_distribution<> {0,1});
    m_normalDistros.push_back(std::normal_distribution<> {0,1});

    // create a mersenne twister engine for each spatial dimension.
    // the engines will be used by the SND-RNGs.
    m_mtEngines.push_back(std::mt19937 {rd()});
    m_mtEngines.push_back(std::mt19937 {rd()});
    m_mtEngines.push_back(std::mt19937 {rd()});
}

Steering Behavior_GaussMarkovRandomWalk::apply()
{
    // store the current state in the average
    updateStateAverage();

    // compute the next state
    Vector3d nextVel = getAgent()->getVelocity() * m_independenceLevel +
            m_stateAverage * (1 - m_independenceLevel) +
            generateNoiseVector() * m_gaussRandomNoiseVariance *sqrt(1-m_independenceLevel*m_independenceLevel);


    // generate steering to match state
    return Steering(
                Behavior_SelfAlignment(m_agentId).alignOnVelocity(getAgent()->getVelocity()),
                Vector3d(nextVel - getAgent()->getVelocity())
                );
}

Vector3d Behavior_GaussMarkovRandomWalk::generateNoiseVector()
{
    return Vector3d(
                m_normalDistros.at(0)(m_mtEngines.at(0)),
                m_normalDistros.at(1)(m_mtEngines.at(1)),
                m_normalDistros.at(2)(m_mtEngines.at(2))
                );
}

void Behavior_GaussMarkovRandomWalk::updateStateAverage()
{
    Vector3d currentVel = getAgent()->getVelocity();
    m_stateAverage.x = (m_stateAverage.x * m_stateCount + currentVel.x) /
            (m_stateCount + 1);
    m_stateAverage.y = (m_stateAverage.y * m_stateCount + currentVel.y) /
            (m_stateCount + 1);
    m_stateAverage.z = (m_stateAverage.z * m_stateCount + currentVel.z) /
            (m_stateCount + 1);
    m_stateCount++;
}

} // namespace LIMoSIM
