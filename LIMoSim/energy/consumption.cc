#include "consumption.h"

namespace LIMoSim {
namespace Energy {

Consumption::Consumption():
    m_energy_J(0),
    m_timestamp_s(0)
{

}

double Consumption::getEnergy()
{
    return m_energy_J;
}

double Consumption::getTimestamp()
{
    return m_timestamp_s;
}

void Consumption::increment(double _timeDelta_s, double _energy_J)
{
    m_timestamp_s += _timeDelta_s;
    m_energy_J +=_energy_J;
}

} // namespace Energy
} // namespace LIMoSim
