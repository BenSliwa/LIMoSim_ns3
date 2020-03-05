/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 CTTC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Nicola Baldo <nbaldo@cttc.es>
 */

#include <ns3/mobility-model.h>
#include <cmath> // for M_PI
#include "LIMoSim/world/raytracing/raytracing.h"
#include "ns3/ns3utils.h"
#include "math.h"


#include <algorithm>
#include "deterministic-obstacle-shadowing-spectrum-propagation-loss.h"
#include "obstacleshadowingcache.h"

namespace LIMoSim{
namespace NS3 {

using namespace ns3;
using namespace Modules::ObstacleShadowing;

NS_OBJECT_ENSURE_REGISTERED (DeterministicObstacleShadowingSpectrumPropagationLossModel);


DeterministicObstacleShadowingSpectrumPropagationLossModel::DeterministicObstacleShadowingSpectrumPropagationLossModel () :
    m_beta(9),
    m_gamma(0.4),
    rayTracing(new Raytracing())
{
}

DeterministicObstacleShadowingSpectrumPropagationLossModel::~DeterministicObstacleShadowingSpectrumPropagationLossModel ()
{
}


TypeId
DeterministicObstacleShadowingSpectrumPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::DeterministicObstacleShadowingSpectrumPropagationLossModel")
    .SetParent<SpectrumPropagationLossModel> ()
    .SetGroupName ("Spectrum")
    .AddConstructor<DeterministicObstacleShadowingSpectrumPropagationLossModel> ()
  ;
  return tid;
}

Ptr<SpectrumValue>
DeterministicObstacleShadowingSpectrumPropagationLossModel::DoCalcRxPowerSpectralDensity (Ptr<const SpectrumValue> txPsd,
                                                                 Ptr<const MobilityModel> a,
                                                                 Ptr<const MobilityModel> b) const
{
  Ptr<SpectrumValue> rxPsd = Copy<SpectrumValue> (txPsd);
  Values::iterator vit = rxPsd->ValuesBegin ();
  Bands::const_iterator fit = rxPsd->ConstBandsBegin ();

  NS_ASSERT (a);
  NS_ASSERT (b);

  double d = a->GetDistanceFrom (b);
  static auto osc = ObstacleShadowingCache::getInstance();

  while (vit != rxPsd->ValuesEnd ())
    {

      NS_ASSERT (fit != rxPsd->ConstBandsEnd ());

      const Vector _a = a->GetPosition();
      const Vector _b = b->GetPosition();

      std::string cacheKeyVal = toObstacleShadowingCacheKey(_a, _b);

      double L_mW;
      if(osc->counts(cacheKeyVal, &L_mW)) {
         *vit /= L_mW;
         ++vit;
         ++fit;
      } else {

          Vector3d pos_a = toLIMoSimVector(a->GetPosition());
          Vector3d pos_b = toLIMoSimVector(b->GetPosition());
          RayTrace trace = rayTracing->trace(pos_a, pos_b);
          double L_obs_dB = trace.attenuated_m * m_gamma + trace.intersections.size() * m_beta;
          double L_obs_mW = pow(10, L_obs_dB/10);
          double L_mW = CalculateLoss (fit->fc, d);

    //      std::cout << "loss " << L_obs_dB << "\t" << 10*log10(L_mW) << "\t"
    //                << pos_a.toString() << "\t"
    //                << pos_b.toString() << std::endl;

          L_mW += L_obs_mW;
          osc->cache(cacheKeyVal, L_mW);
          *vit /= L_mW;
          ++vit;
          ++fit;
      }

    }
  return rxPsd;
}


double
DeterministicObstacleShadowingSpectrumPropagationLossModel::CalculateLoss (double f, double d) const
{
  NS_ASSERT (d >= 0);

  if (std::abs(d) < 1e-6)
    {
      return 1;
    }

  NS_ASSERT (f > 0);
  double loss_sqrt = (4 * M_PI * f * d) / 3e8;
  double loss = loss_sqrt * loss_sqrt;

  if (loss < 1)
    {
      loss = 1;
    }
  return loss;
}


}  // namespace NS3
}  // namespace LIMoSIm
