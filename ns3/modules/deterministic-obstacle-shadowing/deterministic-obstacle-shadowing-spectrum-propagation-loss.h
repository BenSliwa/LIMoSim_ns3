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

#ifndef DETERMINISTIC_OBSTACLE_SHADOWING_SPECTRUM_PROPAGATION_LOSS_H
#define DETERMINISTIC_OBSTACLE_SHADOWING_SPECTRUM_PROPAGATION_LOSS_H


#include <ns3/spectrum-propagation-loss-model.h>

#include "LIMoSim/world/raytracing/raytracing.h"

namespace ns3 {
class MobilityModel;
}

namespace LIMoSim{
namespace NS3 {
using namespace ns3;

/**
 * \brief Deterministic obstacle shadowing spectrum propagation loss model
 *
 */
class DeterministicObstacleShadowingSpectrumPropagationLossModel : public SpectrumPropagationLossModel
{

public:
  DeterministicObstacleShadowingSpectrumPropagationLossModel ();
  ~DeterministicObstacleShadowingSpectrumPropagationLossModel ();

  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId ();


  virtual Ptr<SpectrumValue> DoCalcRxPowerSpectralDensity (Ptr<const SpectrumValue> txPsd,
                                                           Ptr<const MobilityModel> a,
                                                           Ptr<const MobilityModel> b) const;


  /**
   * Return the propagation loss L according to a simplified version of Friis'
   * formula in which antenna gains are unitary
   *
   * @param f frequency in Hz
   * @param d distance in m
   *
   * @return if Prx < Ptx then return Prx; else return Ptx
   */
  double CalculateLoss (double f, double d) const;


private:
  double m_beta;
  double m_gamma;
  Raytracing* rayTracing;
};






} // namespace ns3
} // namespace LIMoSim

#endif /* DETERMINISTIC_OBSTACLE_SHADOWING_SPECTRUM_PROPAGATION_LOSS_H */
