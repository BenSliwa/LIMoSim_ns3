#ifndef TRAFFICANIMATION_H
#define TRAFFICANIMATION_H

#include <ns3/application.h>
#include <ns3/network-module.h>

namespace LIMoSim {
namespace NS3 {
namespace Applications {

using namespace ns3;

class TrafficAnimation: public Application
{
public:
    static TypeId GetTypeId (void);
    TrafficAnimation();
    virtual ~TrafficAnimation();

protected:
    virtual void DoDispose();

private:
    virtual void StartApplication (void);
    virtual void StopApplication (void);

    void Send (Ptr<Socket>, uint32_t);
    void Receive (Ptr<Socket> socket);

    Ptr<Socket>     m_RxSocket, m_TxSocket;
    std::string m_nodeId;
    uint16_t m_dlPort, m_ulPort = 2000;


};

} // namespace Applications
} // namespace NS3
} // namespace LIMoSim

#endif // TRAFFICANIMATION_H
