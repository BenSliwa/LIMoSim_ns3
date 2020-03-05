#ifndef MOBILITYRECEIVERUDP_H
#define MOBILITYRECEIVERUDP_H

#include <ns3/application.h>
#include <ns3/core-module.h>
#include <ns3/socket.h>

namespace LIMoSim {
namespace NS3 {
namespace Applications {

using namespace ns3;

class MobilityReceiverUDP: public Application
{
public:
    static TypeId GetTypeId (void);
    MobilityReceiverUDP();
    virtual ~MobilityReceiverUDP();

protected:
    virtual void DoDispose (void);

private:
    virtual void StartApplication (void);
    virtual void StopApplication (void);

    void ReceiveMobilityData (Ptr<Socket> socket);

    Ptr<Socket>     m_socket;
    EventId         m_rxTimeoutEvent;

    uint16_t        m_port;

    uint32_t        m_recoverCount;

    std::string m_vehicleId;
};

} // namespace Applications
} // namespace NS3
} // namespace LIMoSim

#endif // MOBILITYRECEIVERUDP_H
