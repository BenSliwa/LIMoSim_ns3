#include "mobilitysenderudp.h"

// ns3
#include <ns3/socket-factory.h>
#include <ns3/udp-socket-factory.h>

// LIMoSim ns3
#include "ns3/limosimmobilitymodel.h"

// LIMoSim
#include "LIMoSim/world/vehiclemanager.h"

namespace LIMoSim {
namespace NS3 {
namespace Applications {

using namespace ns3;

TypeId MobilitySenderUDP::GetTypeId()
{
    static TypeId tid = TypeId ("MobilitySenderUDP")
      .SetParent<Application> ()
      .AddConstructor<MobilitySenderUDP> ()
      .AddAttribute ("PacketSize", "The size of packets transmitted.",
                     UintegerValue (64),
                     MakeUintegerAccessor (&MobilitySenderUDP::m_pktSize),
                     MakeUintegerChecker<uint32_t>(1))
      .AddAttribute ("Destination", "Target host address.",
                     Ipv4AddressValue ("255.255.255.255"),
                     MakeIpv4AddressAccessor (&MobilitySenderUDP::m_destAddr),
                     MakeIpv4AddressChecker ())
      .AddAttribute ("Port", "Destination app port.",
                     UintegerValue (1603),
                     MakeUintegerAccessor (&MobilitySenderUDP::m_destPort),
                     MakeUintegerChecker<uint32_t>())
      .AddAttribute ("Interval", "Delay between transmissions.",
                     StringValue ("ns3::ConstantRandomVariable[Constant=0.5]"),
                     MakePointerAccessor (&MobilitySenderUDP::m_interval),
                     MakePointerChecker <RandomVariableStream>())
      .AddTraceSource ("Tx", "A new packet is created and is sent",
                       MakeTraceSourceAccessor (&MobilitySenderUDP::m_txTrace),
                       "ns3::Packet::TracedCallback")
    ;
    return tid;
}

MobilitySenderUDP::MobilitySenderUDP()
{
    NS_LOG_FUNCTION_NOARGS ();
    m_interval = CreateObject<ConstantRandomVariable> ();
    m_socket = nullptr;
}

MobilitySenderUDP::~MobilitySenderUDP()
{
    NS_LOG_FUNCTION_NOARGS ();
}

void MobilitySenderUDP::DoDispose()
{
    NS_LOG_FUNCTION_NOARGS ();
    m_socket = nullptr;
    // chain up
    Application::DoDispose ();
}

void MobilitySenderUDP::StartApplication()
{
    NS_LOG_FUNCTION_NOARGS ();

    if (m_socket == nullptr) {
        Ptr<SocketFactory> socketFactory = GetNode ()->GetObject<SocketFactory>
            (UdpSocketFactory::GetTypeId ());
        m_socket = socketFactory->CreateSocket ();
        m_socket->Bind ();
      }

    m_count = 0;

    Simulator::Cancel (m_sendEvent);
    m_sendEvent = Simulator::ScheduleNow (&MobilitySenderUDP::SendMobilityData, this);

    Ptr<LIMoSim::NS3::LimoSimMobilityModel> mobility = GetNode()->GetObject<LIMoSim::NS3::LimoSimMobilityModel>();
    if (!mobility) {
        std::cerr << "LIMoSim::NS3::Applications::MobilitySenderUDP: could not retrieve LIMoSim mobility model." << std::endl;
        exit(1);
    }
    m_vehicleId = mobility->GetVehicleId();
}

void MobilitySenderUDP::StopApplication()
{
    NS_LOG_FUNCTION_NOARGS ();
    Simulator::Cancel (m_sendEvent);
}

void MobilitySenderUDP::SendMobilityData()
{
    NS_LOG_INFO ("Sending packet at " << Simulator::Now () << " to " <<
                 m_destAddr);


    // Retrieve mobility data of LIMoSim vehicle
    Vehicle *v = VehicleManager::getInstance()->getVehicle(m_vehicleId);
    MobilityData data = v->getLocalVehicleManager()->getVehicleMobiliyData();

    // Convert mobility data to byte array
    std::stringstream ss;
    data.serialize(ss);
    std::string data_serial = ss.str();
    uint64_t data_size = data_serial.size();
    uint8_t *p = new uint8_t[data_size];
    memset(p,0,data_size);
    memcpy(p, data_serial.c_str(), data_size);
//    std::cout << "sent buffer: " << (char* )p << std::endl;

    // Create packet holding mobility data
    Ptr<Packet> packet = Create<Packet>((p), data_size);


    // Could connect the socket since the address never changes; using SendTo
    // here simply because all of the standard apps do not.
    m_socket->SendTo (packet, 0, InetSocketAddress (m_destAddr, m_destPort));
    delete[] p;

    // Report the event to the trace.
    m_txTrace (packet);

    m_sendEvent = Simulator::Schedule (Seconds (m_interval->GetValue ()),
                                       &MobilitySenderUDP::SendMobilityData, this);

}

} // namespace Applications
} // namespace NS3
} // namespace LIMoSim
