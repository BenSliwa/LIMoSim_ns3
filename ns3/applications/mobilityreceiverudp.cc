#include "mobilityreceiverudp.h"
#include <strstream>


#include <ns3/socket-factory.h>
#include <ns3/udp-socket-factory.h>

#include "ns3/limosimmobilitymodel.h"

#include "LIMoSim/world/vehiclemanager.h"

namespace LIMoSim {
namespace NS3 {
namespace Applications {

using namespace ns3;

TypeId MobilityReceiverUDP::GetTypeId()
{
    static TypeId tid = TypeId ("MobilityReceiverUDP")
      .SetParent<Application> ()
      .AddConstructor<MobilityReceiverUDP> ()
      .AddAttribute ("Port", "Listening port.",
                     UintegerValue (1603),
                     MakeUintegerAccessor (&MobilityReceiverUDP::m_port),
                     MakeUintegerChecker<uint32_t>())
    ;
    return tid;
}

MobilityReceiverUDP::MobilityReceiverUDP()
{
    NS_LOG_FUNCTION_NOARGS ();
    m_socket = nullptr;
}

MobilityReceiverUDP::~MobilityReceiverUDP()
{
    NS_LOG_FUNCTION_NOARGS ();
}

void MobilityReceiverUDP::DoDispose()
{
    NS_LOG_FUNCTION_NOARGS ();

    m_socket = nullptr;
    // chain up
    Application::DoDispose ();
}

void MobilityReceiverUDP::StartApplication()
{
    NS_LOG_FUNCTION_NOARGS ();

    if (m_socket == nullptr) {
        Ptr<SocketFactory> socketFactory = GetNode ()->GetObject<SocketFactory>
            (UdpSocketFactory::GetTypeId ());
        m_socket = socketFactory->CreateSocket ();
        InetSocketAddress local =
          InetSocketAddress (Ipv4Address::GetAny (), m_port);
        m_socket->Bind (local);
      }

    m_socket->SetRecvCallback (MakeCallback (&MobilityReceiverUDP::ReceiveMobilityData, this));

    Ptr<LIMoSim::NS3::LimoSimMobilityModel> mobility = GetNode()->GetObject<LIMoSim::NS3::LimoSimMobilityModel>();
    if (!mobility) {
        std::cerr << "LIMoSim::NS3::Applications::MobilityReceiverUDP: could not retrieve LIMoSim mobility model." << std::endl;
        exit(1);
    }
    m_vehicleId = mobility->GetVehicleId();
}

void MobilityReceiverUDP::StopApplication()
{
    NS_LOG_FUNCTION_NOARGS ();

    if (m_socket) {
        m_socket->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
    }
}

void MobilityReceiverUDP::ReceiveMobilityData(Ptr<Socket> socket)
{
    // NS_LOG_FUNCTION (this << socket << packet << from);

    Ptr<Packet> packet;
    Address from;
    while ((packet = socket->RecvFrom (from))) {

        // reception animation for the UI
        // UI::AnimationUtils::animateReception(m_vehicleId);

        if (InetSocketAddress::IsMatchingType (from)) {
            NS_LOG_INFO ("Received " << packet->GetSize () << " bytes from " <<
                         InetSocketAddress::ConvertFrom (from).GetIpv4 ());
//            std::cout << "Received " << packet->GetSize () << " bytes from " <<
//                                   InetSocketAddress::ConvertFrom (from).GetIpv4 () << std::endl;

            MobilityData data ("");

            // retrieve received buffer
            uint8_t *p = new uint8_t [packet->GetSize()];
            memset(p,0,packet->GetSize());
            packet->CopyData(p, packet->GetSize());
//            std::cout << "received buffer: " << (char*) p << std::endl;


            std::stringstream in (reinterpret_cast<char*>(p));
            data.deserialize(in);
            delete[] p;

//            std::cout <<"received mobility data from vehicle: " << data.name;
//            std::cout <<" with position: " << data.position.toString() << std::endl;
            VehicleManager::getInstance()->getVehicle(m_vehicleId)
                    ->getLocalVehicleManager()->updateMobilityDataReport(data);

        }
    }
}

} // namespace Applications
} // namespace NS3
} // namespace LIMoSim
