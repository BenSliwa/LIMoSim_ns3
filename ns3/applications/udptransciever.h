#ifndef UDPTRANSCIEVER_H
#define UDPTRANSCIEVER_H

#include <ns3/application.h>
#include <ns3/socket.h>

#include "LIMoSim/utils/typedefs.h"

namespace LIMoSim {
namespace NS3 {
namespace Applications {

using namespace ns3;
using namespace utils::typedefs;

class UdpTransciever: public Application
{
public:
    static TypeId GetTypeId (void);

    UdpTransciever();

    void setDestinationAddresses(StringVector _destinatioAddrs);
    void setReceiveCallback(Callback<void, std::string, uint> _rxCallback);
    void send(std::string _msg);
    void send(std::string _msg, uint _destIndex);

protected:
    void Setup(Ptr<Socket> _socket);

    // Application interface
protected:
    virtual void DoDispose (void);
private:
    virtual void StartApplication (void);
    virtual void StopApplication (void);

    // Transciever logic
    void receiveMsg (Ptr<Socket> _socket);
    void sendMsg (std::string _msg);
    void sendMsg (std::string _msg, uint _destIndex);

private:
//    Ipv4Address                 m_destAddr;
    uint16_t                    m_destPort;
    Ptr<Socket>                 m_socket;
    Callback<void, std::string, uint> m_rxCallback;
    StringVector                m_destAddrs;

};

} // namespace Applications
} // namespace NS3
} // namespace LIMoSim

#endif // UDPTRANSCIEVER_H
