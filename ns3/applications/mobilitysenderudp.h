#ifndef MOBILITYSENDERUDP_H
#define MOBILITYSENDERUDP_H

// ns3
#include <ns3/application.h>
#include <ns3/core-module.h>
#include <ns3/socket.h>

namespace LIMoSim {
namespace NS3 {
namespace Applications {

using namespace ns3;

class MobilitySenderUDP: public Application
{
public:
    static TypeId GetTypeId (void);
    MobilitySenderUDP();
    virtual ~MobilitySenderUDP();

protected:
    virtual void DoDispose (void);

private:
    virtual void StartApplication (void);
    virtual void StopApplication (void);

    void SendMobilityData ();

    uint32_t        m_pktSize;
    Ipv4Address     m_destAddr;
    uint16_t        m_destPort;
    Ptr<ConstantRandomVariable> m_interval;
    uint32_t        m_numPkts;

    Ptr<Socket>     m_socket;
    EventId         m_sendEvent;

    TracedCallback<Ptr<const Packet> > m_txTrace;

    uint32_t        m_count;
    std::string     m_vehicleId;
};

} // namespace Applications
} // namespace NS3
} // namespace LIMoSim

#endif // MOBILITYSENDERUDP_H
