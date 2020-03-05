#include "trafficanimation.h"

#include <ns3/log.h>
#include <ns3/internet-module.h>

#include "ns3/ns3utils.h"

namespace LIMoSim {
namespace NS3 {
namespace Applications {

TypeId TrafficAnimation::GetTypeId()
{
    static TypeId tid = TypeId ("TrafficAnimation")
            .SetParent<Application> ()
            .AddConstructor<TrafficAnimation> ()
            .AddAttribute ("DlPort", "Downlink port.",
                           UintegerValue (1603),
                           MakeUintegerAccessor (&TrafficAnimation::m_dlPort),
                           MakeUintegerChecker<uint16_t>());
    return tid;
}

TrafficAnimation::TrafficAnimation()
{
    NS_LOG_FUNCTION_NOARGS ();
    m_RxSocket = 0;
    m_TxSocket = 0;
}

TrafficAnimation::~TrafficAnimation()
{
    NS_LOG_FUNCTION_NOARGS ();
}

void TrafficAnimation::DoDispose()
{
    NS_LOG_FUNCTION_NOARGS ();

    m_RxSocket = 0;
    m_TxSocket = 0;
    // chain up
    Application::DoDispose ();
}

void TrafficAnimation::StartApplication()
{
    NS_LOG_FUNCTION_NOARGS ();
    {
        Ptr<SocketFactory> socketFactory = GetNode ()->GetObject<SocketFactory>
            (UdpSocketFactory::GetTypeId ());
        Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4> ();
        Ipv4InterfaceAddress iaddr = ipv4->GetAddress (1,0);
        Ipv4Address ipAddr = iaddr.GetLocal ();

        if (m_RxSocket == 0) {
            m_RxSocket = socketFactory->CreateSocket ();
            InetSocketAddress local = InetSocketAddress (ipAddr, m_dlPort);
            m_RxSocket->Bind (local);
        }


        if (m_TxSocket == 0) {
            m_TxSocket = socketFactory->CreateSocket ();
            //InetSocketAddress local = InetSocketAddress (ipAddr, m_ulPort);
            m_TxSocket->Bind ();
        }
    }


    m_nodeId = static_cast<std::ostringstream*>( &(std::ostringstream() << GetNode()->GetId()) )->str();

    m_TxSocket->SetDataSentCallback(MakeCallback (&TrafficAnimation::Send, this));
    m_RxSocket->SetRecvCallback (MakeCallback (&TrafficAnimation::Receive, this));

}

void TrafficAnimation::StopApplication()
{
    NS_LOG_FUNCTION_NOARGS ();

    if (m_RxSocket != 0) {
        m_RxSocket->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
    }

    if (m_TxSocket != 0) {
        m_TxSocket->SetDataSentCallback(MakeNullCallback<void, Ptr<Socket>, uint32_t> ());
    }
}

void TrafficAnimation::Send(Ptr<Socket>, uint32_t)
{
    std::cout << "sending" <<std::endl;
    UI::AnimationUtils::animateTransmission(m_nodeId);
}

void TrafficAnimation::Receive(Ptr<Socket> socket)
{

    // std::cout << m_nodeId << " is receiving" <<std::endl;
    Ptr<Packet> packet;
    Address from;
    while ((packet = socket->RecvFrom (from))) {
        UI::AnimationUtils::animateReception(m_nodeId);
    }
}

} // namespace Applications
} // namespace NS3
} // namespace LIMoSim
