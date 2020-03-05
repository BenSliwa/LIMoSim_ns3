#ifndef CONSUMPTION_H
#define CONSUMPTION_H

namespace LIMoSim {
namespace Energy {

/**
 * @brief The Consumption class
 * Keeps track of the cumulated energy consumption of vehicles.
 */
class Consumption
{
public:
    Consumption();

    double getEnergy();
    double getTimestamp();

    void increment(double _timeDelta_s, double _energy_J);

private:
    double m_energy_J;
    double m_timestamp_s;
};

} // namespace Energy
} // namespace LIMoSim

#endif // CONSUMPTION_H
