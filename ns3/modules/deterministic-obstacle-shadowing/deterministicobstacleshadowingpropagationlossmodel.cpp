#include "deterministicobstacleshadowingpropagationlossmodel.h"

#include "math.h"

#include "obstacleshadowingcache.h"
#include "ns3/ns3utils.h"


#include <ns3/mobility-model.h>
#include <ns3/log.h>

namespace LIMoSim {
namespace NS3 {

using namespace ns3;
using namespace  Modules::ObstacleShadowing;

NS_OBJECT_ENSURE_REGISTERED (DeterministicObstacleShadowingPropagationLossModel);


DeterministicObstacleShadowingPropagationLossModel::
DeterministicObstacleShadowingPropagationLossModel():
    rayTracing(new Raytracing())
{

}

DeterministicObstacleShadowingPropagationLossModel::~DeterministicObstacleShadowingPropagationLossModel()
{

}

TypeId DeterministicObstacleShadowingPropagationLossModel::GetTypeId()
{
    static TypeId tid = TypeId ("ns3::DeterministicObstacleShadowingPropagationLossModel")
      .SetParent<FriisPropagationLossModel> ()
      .SetGroupName("Propagation")
      .AddConstructor<DeterministicObstacleShadowingPropagationLossModel> ()
    ;
    return tid;
}

double DeterministicObstacleShadowingPropagationLossModel::GetLoss(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const
{
    double d = a->GetDistanceFrom (b);
    static auto osc = ObstacleShadowingCache::getInstance();
    const Vector _a = a->GetPosition();
    const Vector _b = b->GetPosition();

    std::string cacheKeyVal = toObstacleShadowingCacheKey(_a, _b);
    double L_mW;
    if (osc->counts(cacheKeyVal, &L_mW)){
        return L_mW;
    } else {
        Vector3d pos_a = toLIMoSimVector(_a);
        Vector3d pos_b = toLIMoSimVector(_b);RayTrace trace = rayTracing->trace(pos_a, pos_b);
        double L_obs_dB = trace.attenuated_m * m_gamma + trace.intersections.size() * m_beta;
        double L_obs_mW = pow(10, L_obs_dB/10);
        double L_mW = CalculateLoss (d);
        L_mW += L_obs_mW;
        osc->cache(cacheKeyVal, L_mW);
        return L_mW;
    }
}

double DeterministicObstacleShadowingPropagationLossModel::DoCalcRxPower(double txPowerDbm, Ptr<MobilityModel> a, Ptr<MobilityModel> b) const
{
    return txPowerDbm - GetLoss(a,b);
}

double DeterministicObstacleShadowingPropagationLossModel::CalculateLoss(double distance) const
{
    NS_ASSERT (distance >= 0);

    double lambda = GetWavelength();
    if (distance < 3*lambda)
         {
           NS_LOG_WARN ("distance not within the far field region => inaccurate propagation loss value");
         }
       if (distance <= 0)
         {
           return GetMinLoss();
         }
       double numerator = lambda * lambda;
       double denominator = 16 * M_PI * M_PI * distance * distance * GetSystemLoss();
       double lossDb = -10 * log10 (numerator / denominator);
       NS_LOG_DEBUG ("distance=" << distance<< "m, loss=" << lossDb <<"dB");
       return std::max (lossDb, GetMinLoss());
}

double DeterministicObstacleShadowingPropagationLossModel::GetWavelength() const
{
    static const double C = 299792458.0;
    return C / GetFrequency();
}

} // namespace NS3
} // namespace LIMoSim
