#include "udptransciever.h"

#include <ns3/core-module.h>
#include <ns3/udp-socket-factory.h>

#include "LIMoSim/utils/vector.h"

namespace LIMoSim {
namespace NS3 {
namespace Applications {

NS_LOG_COMPONENT_DEFINE ("UdpTransciever");
NS_OBJECT_ENSURE_REGISTERED (UdpTransciever);

TypeId UdpTransciever::GetTypeId()
{
    static TypeId tid = TypeId ("UdpTransciever")
            .SetParent<Application> ()
            .AddConstructor<UdpTransciever> ()
            .AddAttribute ("Port", "Destination app port.",
                           UintegerValue (8080),
                           MakeUintegerAccessor (&UdpTransciever::m_destPort),
                           MakeUintegerChecker<uint16_t>())
    ;
    return tid;
}

UdpTransciever::UdpTransciever():
    m_socket(nullptr)
{

}

void UdpTransciever::setDestinationAddresses(StringVector _destinatioAddrs)
{
    m_destAddrs = _destinatioAddrs;
}

void UdpTransciever::setReceiveCallback(Callback<void, std::string, uint> _rxCallback)
{
    NS_LOG_FUNCTION (this << &_rxCallback);
    m_rxCallback = _rxCallback;
}

void UdpTransciever::send(std::string _msg)
{
    this->sendMsg(_msg);
}

void UdpTransciever::send(std::string _msg, uint _destIndex)
{
    this->sendMsg(_msg, _destIndex);
}

void UdpTransciever::Setup(Ptr<Socket> _socket)
{
    NS_LOG_FUNCTION(_socket);
    m_socket = _socket;
}

void UdpTransciever::DoDispose()
{
    NS_LOG_FUNCTION_NOARGS ();
    m_socket = nullptr;
    Application::DoDispose ();
}

void UdpTransciever::StartApplication()
{
    NS_LOG_FUNCTION_NOARGS ();
    if (m_socket == nullptr) {
        Ptr<SocketFactory> socketFactory = GetNode ()->GetObject<SocketFactory>
            (UdpSocketFactory::GetTypeId ());
        m_socket = socketFactory->CreateSocket ();
        InetSocketAddress local =
        InetSocketAddress (Ipv4Address::GetAny(), m_destPort);
        if (m_socket->Bind(local) == -1) {
            std::cerr << "UdpTransciever: Failed to bind socket to " << GetNode()->GetId() << ":" << m_destPort << std::endl;
        }
//        if (m_socket->Connect(InetSocketAddress(m_destAddr,m_destPort)) == -1) {
//            std::cerr << "UdpTransciever: could not connect to remote host" << ss.str() << ":" << m_destPort << std::endl;
//        }
        m_socket->SetAllowBroadcast(true);
        m_socket->SetRecvCallback(MakeCallback (&UdpTransciever::receiveMsg, this));
    }
}

void UdpTransciever::StopApplication()
{
    NS_LOG_FUNCTION_NOARGS ();
}

void UdpTransciever::receiveMsg(Ptr<Socket> _socket)
{
    Ptr<Packet> packet;
    Address from;
    std::cout << "receiving packet" << std::endl;
    while ((packet = _socket->RecvFrom(from))) {
        NS_LOG_INFO(this  << " (UdpTransciever) Received ParcelDeliveryMsg ["
                    << packet->GetSize() <<" bytes] from "
                    << InetSocketAddress::ConvertFrom (from).GetIpv4 ());

        // Allocation reception buffer with one additional char
        // for string termination character
        uint8_t *packetData = new uint8_t [packet->GetSize()+1];
        memset(packetData,0,packet->GetSize()+1);
        packet->CopyData(packetData, packet->GetSize());
        std::string msg = reinterpret_cast<char*>(packetData);

        std::cout << "(UdpTransciever) decoded message: " << msg << std::endl;

        delete[]packetData;

//        if (msg != std::string("ACK")) {
//            this->sendMsg("ACK");
//        }

        std::ostringstream os;
        os << InetSocketAddress::ConvertFrom (from).GetIpv4 ();
        std::string sourceIp = os.str();
        int sourceIpIndex = utils::vector::indexOf(m_destAddrs, sourceIp);
        if (sourceIpIndex != -1) {
            this->m_rxCallback(msg, static_cast<uint>(sourceIpIndex));
        } else {
            std::cout << "UdpTransciever::receiveMsg coudl not retreive sender ip"
                      << sourceIp <<"in destination list." << std::endl;
        }
    }
}

void UdpTransciever::sendMsg(std::string _msg)
{
    if (m_socket){
        NS_LOG_INFO ("(UavDelivery) Sending parcel delivery msg" << _msg
                     << " at " << Simulator::Now ()
                     << " to " << m_destAddrs[0]
                     );

        // Prepare packet
        size_t msgSize = _msg.length();
        uint8_t *packetData = new uint8_t[msgSize];
        memset(packetData,0,msgSize);
        memcpy(packetData, _msg.c_str(), msgSize);
        Ptr<Packet> packet = Create<Packet>(packetData, msgSize);
        std::cout << "sending packet" << std::endl;
        m_socket->SendTo(packet, 0, InetSocketAddress(Ipv4Address(m_destAddrs[0].c_str()), m_destPort));
        // Clean up
        delete[] packetData;
    }
}

void UdpTransciever::sendMsg(std::string _msg, uint _destIndex)
{
    if (_destIndex > m_destAddrs.size()) {
        std::cerr << "UdpTransciever::sendMsg destination address index "
                  << _destIndex << "out of range." << std::endl;
        return;
    }
    if (m_socket){

        auto destAddr = Ipv4Address(m_destAddrs[_destIndex].c_str());
        NS_LOG_INFO ("(UavDelivery) Sending parcel delivery msg" << _msg
                     << " at " << Simulator::Now ()
                     << " to " << m_destAddrs[_destIndex]
                     );

        // Prepare packet
        size_t msgSize = _msg.length();
        uint8_t *packetData = new uint8_t[msgSize];
        memset(packetData,0,msgSize);
        memcpy(packetData, _msg.c_str(), msgSize);
        Ptr<Packet> packet = Create<Packet>(packetData, msgSize);
        std::cout << "sending packet" << std::endl;
        m_socket->SendTo(packet, 0, InetSocketAddress(destAddr, m_destPort));
        // Clean up
        delete[] packetData;
    }
}

} // namespace Applications
} // namespace NS3
} // namespace LIMoSim
