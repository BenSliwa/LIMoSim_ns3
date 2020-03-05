#ifndef LIMOSIM_NS3_DETERMINISTICOBSTACLESHADOWINGPROPAGATIONLOSSMODEL_H
#define LIMOSIM_NS3_DETERMINISTICOBSTACLESHADOWINGPROPAGATIONLOSSMODEL_H

#include <ns3/propagation-loss-model.h>
#include "LIMoSim/world/raytracing/raytracing.h"

namespace LIMoSim {
namespace NS3 {

using namespace ns3;

class DeterministicObstacleShadowingPropagationLossModel: public FriisPropagationLossModel
{
public:
    DeterministicObstacleShadowingPropagationLossModel();
    ~DeterministicObstacleShadowingPropagationLossModel() override;

    static TypeId GetTypeId ();
    double GetLoss(Ptr<MobilityModel> a,
                   Ptr<MobilityModel> b) const;

private:
    virtual double DoCalcRxPower(double txPowerDbm,
                                 Ptr<MobilityModel> a,
                                 Ptr<MobilityModel> b) const override;/**
    * Return the propagation loss L according to a simplified version of Friis'
    * formula in which antenna gains are unitary
    *
    * @param f frequency in Hz
    * @param d distance in m
    *
    * @return if Prx < Ptx then return Prx; else return Ptx
    */
    double CalculateLoss (double d) const;

    double GetWavelength() const;

    double m_beta;
    double m_gamma;
    Raytracing* rayTracing;
};

} // namespace NS3
} // namespace LIMoSim

#endif // LIMOSIM_NS3_DETERMINISTICOBSTACLESHADOWINGPROPAGATIONLOSSMODEL_H
