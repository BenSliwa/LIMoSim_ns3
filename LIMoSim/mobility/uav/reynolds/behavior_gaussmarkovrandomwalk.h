#ifndef BEHAVIOR_GAUSSMARKOVRANDOMWALK_H
#define BEHAVIOR_GAUSSMARKOVRANDOMWALK_H

#include "LIMoSim/mobility/uav/reynolds/behavior.h"
#include <vector>
#include <random>

namespace LIMoSim {

/**
 * @brief The Behavior_GaussMarkovRandomWalk class
 * Refers to the Gauss-Markov Random Mobility Model in:
 *
 * [1] J. Xie, Y. Wan, J. H. Kim, S. Fu, and K. Namuduri,
 * “A survey and analysis of mobility models for airborne networks,”
 * IEEE Commun. Surv. Tutorials, vol. 16, no. 3, pp. 1221–1238, 2014.
 * https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6689509
 *
 */
class Behavior_GaussMarkovRandomWalk : public Behavior
{
public:
    Behavior_GaussMarkovRandomWalk(double _independenceLevel = 0.95, double _gaussRandomNoiseVariance = 1, std::string _agentId="");

    // Behavior interface
public:
    Steering apply();

private:
    double m_independenceLevel;
    double m_gaussRandomNoiseVariance;
    Vector3d m_stateAverage;
    unsigned int m_stateCount;
    std::vector<std::mt19937> m_mtEngines;
    std::vector<std::normal_distribution<>> m_normalDistros;

    Vector3d generateNoiseVector();
    void updateStateAverage();
};

} // namespace LIMoSIM

#endif // BEHAVIOR_GAUSSMARKOVRANDOMWALK_H
