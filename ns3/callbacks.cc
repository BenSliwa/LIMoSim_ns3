#include "callbacks.h"

#include <iostream>

// ns3
#include <ns3/simulator.h>

#include "LIMoSim/simulation/simulation.h"


namespace LIMoSim {
namespace NS3 {
namespace Callbacks {

using namespace ns3;

void PrintProgress(uint32_t s_period)
{
    std::cout << Simulation::getInstance()->getName() <<"_r" << Simulation::getInstance()->getRunCount()
              << ": " <<  Simulator::Now ().GetSeconds () << " s" << std::endl;
    Simulator::Schedule(Seconds(s_period),&PrintProgress, s_period);
}

} // namespace Callbacks
} // namespace NS3
} // namespace LIMoSim
