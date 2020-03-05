#ifndef LIMOSIM_NS3_MODULES_NETSTATS_WIFINETSTATSCALCULATOR_H
#define LIMOSIM_NS3_MODULES_NETSTATS_WIFINETSTATSCALCULATOR_H

#include <stdint.h>
#include <fstream>
#include <map>

#include "ns3/nstime.h"
#include "ns3/event-id.h"
#include "ns3/ipv4-address.h"
#include "ns3/type-id.h"
#include "ns3/basic-data-calculators.h"

namespace LIMoSim {
namespace NS3 {
namespace Modules {
namespace NetStats {

using namespace ns3;

typedef std::pair<Ipv4Address, Ipv4Address> Ipv4Pair;
/// Container:(Ipv4,Ipv4) pair, uint32t
typedef std::map<Ipv4Pair, uint32_t> Uint32Map;
/// Container:(Ipv4,Ipv4) pair, uint64t
typedef std::map<Ipv4Pair, uint64_t> Uint64Map;
typedef std::map<Ipv4Pair, Ptr<MinMaxAvgTotalCalculator<uint64_t> > > Uint64StatsMap;
/// Container:(Ipv4,Ipv4) pair, double
typedef std::map<Ipv4Pair, double> DoubleMap;

/**
 * @brief The WifiNetStatsCalculator class
 *
 * This class is an ns-3 trace sink that performs the calculation of
 * packet statistics for a wifi network. Statistics are generated
 * on a per ip pair basis.
 *
 * The statistics are calculated at consecutive time windows and
 * periodically written to a file. The calculated statistics are:
 *
 *   - Number of transmitted Packets
 *   - Number of received Packets
 *   - Number of transmitted bytes
 *   - Number of received bytes
 *   - Average, min, max and standard deviation of packet delay (delay is
 *     calculated from the generation of the PDU to its reception)
 */
class WifiNetStatsCalculator: public Object
{
public:
    WifiNetStatsCalculator();

    /**
    *  Register this type.
    *  \return The object TypeId.
    */
    static TypeId GetTypeId (void);

    void DoDispose ();

    std::string GetOutputFilename (void);

    void SetOutputFilename (std::string _outputFilename);

    void SetStartTime (Time t);

    Time GetStartTime () const;

    void SetEpoch (Time e);

    Time GetEpoch () const;

    std::vector<double> GetDelayStats(Ipv4Pair &);

    void TxPacket (Ipv4Address _src,
                Ipv4Address _dst,
                uint64_t _packetSize);

    void RxPacket (Ipv4Address _src,
                Ipv4Address _dst,
                uint64_t _packetSize,
                uint64_t _delay);

private:
    /**
    * Called after each epoch to write collected
    * statistics to output files. During first call
    * it opens output files and write columns descriptions.
    * During next calls it opens output files in append mode.
    */
    void ShowResults (void);
    void WriteResults(std::ofstream& _outFile);

    void ResetResults();
    void RescheduleEndEpoch();
    void EndEpoch();
    EventId m_endEpochEvent;

    Uint32Map m_txPackets; //!< Number of TX Packets by (Ipv4, Ipv4) pair
    Uint32Map m_rxPackets; //!< Number of RX Packets by (Ipv4, Ipv4) pair
    Uint64Map m_txData;  //!< Amount of DL TX Data by (Ipv4, Ipv4) pair
    Uint64Map m_rxData;  //!< Amount of DL RX Data by (Ipv4, Ipv4) pair
    Uint64StatsMap m_delay; //!< DL delay by (Ipv4, Ipv4) pair

    Time m_startTime;
    Time m_epochDuration;
    bool m_firstWrite;
    bool m_pendingOutput;
    std::string m_outputFilename;
};

} // namespace NetStats
} // namespace Modules
} // namespace NS3
} // namespace LIMoSim

#endif // LIMOSIM_NS3_MODULES_NETSTATS_WIFINETSTATSCALCULATOR_H
