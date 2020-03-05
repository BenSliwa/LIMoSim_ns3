#include "wifinetstatscalculator.h"

#include <algorithm>

#include "ns3/object.h"
#include "ns3/log.h"
#include "ns3/config.h"
#include "ns3/string.h"
#include "ns3/simulator.h"

namespace LIMoSim {
namespace NS3 {
namespace Modules {
namespace NetStats {

NS_LOG_COMPONENT_DEFINE ("WifiNetStatsCalculator");

NS_OBJECT_ENSURE_REGISTERED ( WifiNetStatsCalculator);

WifiNetStatsCalculator::WifiNetStatsCalculator():
    m_firstWrite(true),
    m_pendingOutput(false)
{

}

TypeId WifiNetStatsCalculator::GetTypeId()
{
    static TypeId tid =
         TypeId ("LIMoSim::WifiNetStatsCalculator")
         .SetParent<Object>()
         .AddConstructor<WifiNetStatsCalculator> ()
         .SetGroupName("Wifi")
         .AddAttribute ("StartTime", "Start time of the on going epoch.",
                        TimeValue (Seconds (0.)),
                        MakeTimeAccessor (&WifiNetStatsCalculator::SetStartTime,
                                          &WifiNetStatsCalculator::GetStartTime),
                        MakeTimeChecker ())
         .AddAttribute ("EpochDuration", "Epoch duration.",
                        TimeValue (Seconds (0.25)),
                        MakeTimeAccessor (&WifiNetStatsCalculator::GetEpoch,
                                          &WifiNetStatsCalculator::SetEpoch),
                        MakeTimeChecker ())
         .AddAttribute ("OutputFilename",
                        "Name of the file where the downlink results will be saved.",
                        StringValue ("WifiNetStats.txt"),
                        MakeStringAccessor (&WifiNetStatsCalculator::SetOutputFilename),
                        MakeStringChecker ())
       ;
       return tid;
}

void WifiNetStatsCalculator::DoDispose()
{
    if (m_pendingOutput)
     {
        ShowResults ();
     }
}

std::string WifiNetStatsCalculator::GetOutputFilename()
{
    return m_outputFilename;
}

void WifiNetStatsCalculator::SetOutputFilename(std::string _outputFilename)
{
    m_outputFilename = _outputFilename;
}

void WifiNetStatsCalculator::SetStartTime(Time t)
{
    m_startTime = t;
    RescheduleEndEpoch ();
}

Time WifiNetStatsCalculator::GetStartTime() const
{
    return m_startTime;
}

void WifiNetStatsCalculator::SetEpoch(Time e)
{
    m_epochDuration = e;
    RescheduleEndEpoch ();
}

Time WifiNetStatsCalculator::GetEpoch() const
{
    return m_epochDuration;
}

std::vector<double> WifiNetStatsCalculator::GetDelayStats(Ipv4Pair& _p)
{
    std::vector<double> stats;
    Uint64StatsMap::iterator it = m_delay.find(_p);
    if (it == m_delay.end()) {
        stats.push_back(0.0);
        stats.push_back(0.0);
        stats.push_back(0.0);
        stats.push_back(0.0);
        return stats;
    }
    stats.push_back(m_delay[_p]->getMean());
    stats.push_back(m_delay[_p]->getStddev());
    stats.push_back(m_delay[_p]->getMin());
    stats.push_back(m_delay[_p]->getMax());
    return stats;
}

void WifiNetStatsCalculator::TxPacket(Ipv4Address _src, Ipv4Address _dst, uint64_t _packetSize)
{
    Ipv4Pair p (_src, _dst);
    if (Simulator::Now() >= m_startTime){
        m_txPackets[p]++;
        m_txData[p]+= _packetSize;
    }
    m_pendingOutput = true;
}

void WifiNetStatsCalculator::RxPacket(Ipv4Address _src, Ipv4Address _dst, uint64_t _packetSize, uint64_t _delay)
{
    Ipv4Pair p (_src, _dst);
    if (Simulator::Now() >= m_startTime){
        m_rxPackets[p]++;
        m_rxData[p] += _packetSize;

        Uint64StatsMap::iterator it = m_delay.find(p);
        if (it == m_delay.end()) {
            m_delay[p] = CreateObject<MinMaxAvgTotalCalculator<uint64_t> > ();
        }
        m_delay[p]->Update(_delay);
    }
    m_pendingOutput = true;
}

void WifiNetStatsCalculator::ShowResults()
{
    std::ofstream outFile;

    if (m_firstWrite == true) {

        outFile.open (GetOutputFilename().c_str());
        if (!outFile.is_open()) {
            NS_LOG_ERROR ("Can't open file " << GetOutputFilename ().c_str ());
            return;
        }
        m_firstWrite = false;
        outFile << "start\tend\tsrcIp\tdstIp\tTxPkts\tTxBytes\tRxPkts\tRxBytes\t";
        outFile << "delay\tstdDev\tmin\tmax";
        outFile << std::endl;

    } else {
        outFile.open(GetOutputFilename().c_str(), std::ios_base::app);
        if (!outFile.is_open()) {
            NS_LOG_ERROR ("Can't open file " << GetOutputFilename ().c_str ());
            return;
        }
    }
    WriteResults(outFile);
    m_pendingOutput = false;
}

void WifiNetStatsCalculator::WriteResults(std::ofstream &_outFile)
{
    std::vector<Ipv4Pair> pairVector;
    for(Uint32Map::iterator it = m_txPackets.begin();
        it!= m_txPackets.end();
        ++it) {
        if (find(pairVector.begin(),
                 pairVector.end(),
                 (*it).first)
                == pairVector.end()){
            pairVector.push_back((*it).first);
        }
    }

    Time endTime = m_startTime + m_epochDuration;
    for(std::vector<Ipv4Pair>::iterator it = pairVector.begin();
        it != pairVector.end();
        ++it) {
        Ipv4Pair p = *it;

        _outFile << m_startTime.GetNanoSeconds() / 1.0e9 << "\t";
        _outFile << endTime.GetNanoSeconds() / 1.0e9 << "\t";
        _outFile << p.first << "\t";
        _outFile << p.second << "\t";
        _outFile << m_txPackets[p] << "\t";
        _outFile << m_txData[p] << "\t";
        _outFile << m_rxPackets[p] << "\t";
        _outFile << m_rxData[p] << "\t";
        std::vector<double> stats = GetDelayStats(p);
        for (std::vector<double>::iterator it = stats.begin();
             it != stats.end();
             ++it) {
            _outFile << (*it) * 1e-9 << "\t";
        }
        _outFile << std::endl;
    }
    _outFile.close();
}

void WifiNetStatsCalculator::ResetResults()
{
    m_txPackets.erase(m_txPackets.begin(), m_txPackets.end());
    m_rxPackets.erase(m_rxPackets.begin(), m_rxPackets.end());
    m_txData.erase(m_txData.begin(), m_txData.end());
    m_rxData.erase(m_rxData.begin(), m_rxData.end());
    m_delay.erase(m_delay.begin(), m_delay.end());
}

void WifiNetStatsCalculator::RescheduleEndEpoch()
{
    m_endEpochEvent.Cancel();
    ResetResults();
    m_endEpochEvent = Simulator::Schedule(m_epochDuration, &WifiNetStatsCalculator::EndEpoch, this);
}

void WifiNetStatsCalculator::EndEpoch()
{
    ShowResults();
    ResetResults();
    m_startTime += m_epochDuration;
    m_endEpochEvent = Simulator::Schedule(m_epochDuration, &WifiNetStatsCalculator::EndEpoch, this);
}

} // namespace NetStats
} // namespace Modules
} // namespace NS3
} // namespace LIMoSim
