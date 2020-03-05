#include "wavefollower.h"

// std c++
#include<ctime>

// ns3
#include "ns3/core-module.h"
#include "ns3/lte-helper.h"
#include "ns3/lte-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include <ns3/buildings-helper.h>
#include <ns3/point-to-point-helper.h>
#include <ns3/config-store.h>
#include <ns3/yans-wifi-helper.h>
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/node-list.h"

// LIMoSim ns3
#include "ns3/callbacks.h"
#include "ns3/ns3setuphelpers.h"
#include "ns3/limosimmobilitymodel.h"
#include "ns3/tags/timestamptag.h"
#include "ns3/modules/net-stats/wifinetstatscalculator.h"


// LIMoSim
#include "LIMoSim/mobility/uav/reynolds/behaviors.h"
//#include "LIMoSim/mobility/car/car.h"
#include "LIMoSim/world/world.h"
#include "LIMoSim/world/vehiclemanager.h"
#include "LIMoSim/simulation/performancemonitor.h"
//#include "LIMoSim/settings/filehandler.h"
#include "LIMoSim/utils/typedefs.h"

#include "standalone/qclidecorators.h"

#include <QCommandLineParser>
#include <QString>
#include <QStringList>

namespace LIMoSim {
namespace NS3 {
namespace Examples {
namespace WaveFollower {

using namespace ns3;
using Node = ns3::Node;
using namespace LIMoSim::utils::typedefs;


inline std::string exampleName() {
    return "WaveFollower";
}
inline bool monitorPerformance() {
    return false;
}
NS_LOG_COMPONENT_DEFINE (exampleName());

/**
 * @brief nodeIdToIp
 * Maps node ids as strings to ipv4 addresses
 */
std::map<std::string, ns3::Ipv4Address> nodeIdToIp;
/**
 * @brief nodeIdToDestIp
 * Maps node ids as strings to the ipv4address of their destination.
 * As in this scenario each node only sends to one defined destination.
 * This is used to find out the ip addresses of src and dest in send callbacks
 * where only the node id of the sender is available.
 */
std::map<std::string, ns3::Ipv4Address> nodeIdToDestIp;

Ptr<Modules::NetStats::WifiNetStatsCalculator> wifiStats;

uint32_t g_numOfPairs;

bool logInCallbacks = false;

void ReceivePacketInSink(std::string _context,
                         Ptr<const Packet> _packet,
                         const Address& _address) {
    if (logInCallbacks) {
        std::cout << "Received packet in sink" << std::endl;
    //    Header header;
        SeqTsHeader seqTs;
        _packet->PeekHeader(seqTs);
        Time tx = seqTs.GetTs();
        std::cout << "packet delay: " << (Simulator::Now() - tx).GetMicroSeconds() << std::endl;
        std::cout << "packet size: " << _packet->GetSize() << std::endl;
        std::cout << "from: "
                  << InetSocketAddress::ConvertFrom (_address).GetIpv4 ()
                  << std::endl;
    }
}

void ReceivePacketInSinkWithAddresses(std::string _context,
                                      Ptr<const Packet> _packet,
                                      const Address& _srcAddress,
                                      const Address& _dstAddress) {
    SeqTsHeader seqTs;
    _packet->PeekHeader(seqTs);
    Ipv4Header ipv4;
    Time tx = seqTs.GetTs();
    uint64_t delay = (Simulator::Now() - tx).GetNanoSeconds();
    wifiStats->RxPacket(InetSocketAddress::ConvertFrom (_srcAddress).GetIpv4 (),
                        nodeIdToIp.at(extractNodeId(_context)),
                        _packet->GetSize(),
                        delay);
    if (logInCallbacks) {
        std::cout << "Received packet in sink" << std::endl;
    //    Header header;
        std::cout << "packet delay: " << delay * 1e-9 << std::endl;
        std::cout << "packet size: " << _packet->GetSize() << std::endl;
        std::cout << "extracted node id" << extractNodeId(_context) << std::endl;
        std::cout << "from: "
                  << InetSocketAddress::ConvertFrom (_srcAddress).GetIpv4 ()
                  << " to: "
                  << nodeIdToIp.at(extractNodeId(_context))
    //              << InetSocketAddress::ConvertFrom (_dstAddress)
                  << std::endl;
        std::cout << "with Context " << _context << std::endl;
    }
}

void ReceivePacketCallback (std::string _context, Ptr< const Packet > _packet) {
    if (logInCallbacks){
        std::cout << "Received wifi packet at MAC Layer" << std::endl;
    //    SeqTsHeader seqTs;
    //    _packet->PeekHeader(seqTs);
    //    Time tx = seqTs.GetTs();
    //    std::cout << "retrieved delay: " << (Simulator::Now() - tx).GetMilliSeconds() << std::endl;
        Tags::TimestampTag timestamp;
        if(_packet->FindFirstMatchingByteTag(timestamp)) {
            Time tx = Simulator::Now() - timestamp.GetTimestamp();
            std::cout << "retrieved delay: " << tx.GetMicroSeconds() << std::endl;
        }
    }

}

void SendPacketCallback (std::string _context, Ptr<const Packet> _packet) {
    Tags::TimestampTag timestamp;
    timestamp.SetTimestamp(Simulator::Now());
    _packet->AddByteTag(timestamp);
    wifiStats->TxPacket(nodeIdToIp.at(extractNodeId(_context)),
                        nodeIdToDestIp.at(extractNodeId(_context)),
                        _packet->GetSize());
    if (logInCallbacks) {
        std::cout << "Sending wifi packet at MAC Layer" << std::endl;
        std::cout << "from :"
                  << nodeIdToIp.at(extractNodeId(_context))
                  << " to: "
                  << nodeIdToDestIp.at(extractNodeId(_context))
                  << std::endl;
    }
}

void ReceivePacketIpv4L3 (std::string _context,
                          Ptr<const Packet> _packet,
                          Ptr<Ipv4> _address,
                          uint32_t _iface) {
    // compute delay
    Tags::TimestampTag timestamp;
    Time now = Simulator::Now();
    uint64_t delay = 0; // in nanoseconds
    if(_packet->FindFirstMatchingByteTag(timestamp)) {
        Time rxT = now - timestamp.GetTimestamp();
        delay = (rxT).GetNanoSeconds();
    } else {
        std::cerr << "could not retrieve delay in ReceivePacketIpv4L3 with context: "
                  << _context << std::endl;
    }

    // fetch ip header
    Ipv4Header ipv4;
    _packet->PeekHeader(ipv4);

    wifiStats->RxPacket(ipv4.GetSource(),
                        ipv4.GetDestination(),
                        _packet->GetSize(),
                        delay);
    if (logInCallbacks) {
        std::cout << "Received packet in IPv4L3" << std::endl;
        std::cout << "packet delay: " << delay * 1e-9 << std::endl;
        std::cout << "packet size: " << _packet->GetSize() << std::endl;
        std::cout << "extracted node id" << extractNodeId(_context) << std::endl;
        std::cout << "from: "
                  << ipv4.GetSource()
                  << " to: "
                  << ipv4.GetDestination()
                  << std::endl;
        std::cout << "with Context " << _context << std::endl;
    }

}

void SendPacketIpv4L3 (std::string _context, Ptr<const Packet> _packet, Ptr<Ipv4> _address, uint32_t _iface) {
    Tags::TimestampTag timestamp;
    timestamp.SetTimestamp(Simulator::Now());
    _packet->AddPacketTag(timestamp);
    Ipv4Header header;
    _packet->PeekHeader(header);
    header.GetDestination();
    wifiStats->TxPacket(header.GetSource(),
                        header.GetDestination(),
                        _packet->GetSize());
    if (logInCallbacks) {
        std::cout << "Sending wifi packet at IPv4L3" << std::endl;
        std::cout << "from :"
                  << header.GetSource()
                  << " to: "
                  << header.GetDestination()
                  << std::endl;
    }
}

Behavior* behaviorFactory(std::string _targetId) {
//    return new Behavior_FollowAtElevation(_targetId);
    return new Behavior_FollowAtElevation(_targetId, 20.0,40.0,false);
}


int myrand (int _min, int _max) {
    return std::rand() % (_max - _min) + _min;
}

std::vector<VehicleProperties> generateCarProps(uint numberOfUes, std::vector<std::string> colors) {
    std::vector<VehicleProperties> vehiclesProps;
    for (uint i = 0; i < numberOfUes; i++) {
        vehiclesProps.push_back(
                    VehicleProperties(
                        Vector(0,0,0),
                        LIMoSimCar,
                        "C" + std::to_string(i),
                        VehicleUIProps(colors.at(i%colors.size()))
                        )
                    );
    }
    return vehiclesProps;
}

std::vector<VehicleProperties> generateUAVProps(uint numberOfUAVs, std::vector<std::string> colors) {
    std::vector<VehicleProperties> UAVProps;
    Vector3d max = World::getInstance()->getBoxMax(), min = World::getInstance()->getBoxMin();

    for (uint i = 0; i < numberOfUAVs; i++) {

        UAVProps.push_back(
                    VehicleProperties(
//                        Vector(i*50 + 400,300,40),
                        Vector (myrand(min.x, max.x), myrand(min.y, max.y), 40),
                        LIMoSimUAV,
                        "U" + std::to_string(i),
                        VehicleUIProps(colors.at(i%colors.size())),
                        MakeBoundCallback(&behaviorFactory, "C" + std::to_string(i))
                        )
                    );
    }
    return UAVProps;
}



void setup(int _runCount,
           uint16_t _simTime_s,
           uint16_t _numOfPairs,
           uint16_t _interPacketInterval_ms,
           uint16_t _packetSize_B)
{
    LogComponentEnable(exampleName().c_str(), LOG_LEVEL_INFO);
    NS_LOG_INFO("setting up ns3 simulation setup of " << exampleName());
    std::cout << "setting up ns3 simulation setup of " << exampleName() << " run count:" << _runCount <<std::endl;

    setUAVNextToCar(true);

    QCommandLineParser parser;
    parser.setApplicationDescription("LIMoSim - ns3 - lteCoverageMobile");
    parser.addHelpOption();
    parser.addVersionOption();

    Standalone::qclidecorators::addGeneralOptions(parser);
    Standalone::qclidecorators::addSimulationOptions(parser);

    QCommandLineOption simTimeOption(
                QStringList() << "t" << "sim-time",
                "Simulation time in seconds",
                "number");
    QCommandLineOption ueCountOption(
                QStringList() << "n" << "pair-count",
                "Number of user equipment pairs",
                "number");
    QCommandLineOption interPackterIntervalOption(
                "ipi",
                "Interarrival time of packets in milliseconds",
                "number");
    QCommandLineOption packetSizeOption(
                QStringList() << "ps" << "packet-size",
                "Size of the data packets in bytes",
                "number");
    QCommandLineOption runCountOption(
                QStringList() << "r" << "run-count",
                "Run count",
                "number");

    parser.addOptions({
                          simTimeOption,
                          ueCountOption,
                          interPackterIntervalOption,
                          packetSizeOption,
                          runCountOption
                      });
    parser.parse(QCoreApplication::arguments());

    if (parser.isSet("help")) {
        std::cout << parser.helpText().toStdString() << std::endl;
        return;
    }

    if (!parser.errorText().isEmpty()) {
        std::cerr << parser.errorText().toStdString() << std::endl;
        return;
    }


    // Set sim params
    bool ok = true;
    uint16_t simTime_s = parser.isSet(simTimeOption) ?
                parser.value(simTimeOption).toInt(&ok) :
                _simTime_s;
    if (!ok) {
        std::cout<<"NS3::Examples::"<<exampleName()
                <<"::setup: setting simTime fallback value: "
               << _simTime_s <<std::endl;
        simTime_s = _simTime_s;
        ok = true;
    }
    uint16_t numOfPairs = parser.isSet(ueCountOption) ?
                parser.value(ueCountOption).toInt(&ok) :
                _numOfPairs;
    if (!ok) {
        std::cout<<"NS3::Examples::"<<exampleName()
                <<"::setup: setting ue count fallback value: "
               << _numOfPairs <<std::endl;
        numOfPairs = _numOfPairs;
        ok = true;
    }
    uint16_t interPacketInterval_ms = parser.isSet(interPackterIntervalOption) ?
                parser.value(interPackterIntervalOption).toInt(&ok) :
                _interPacketInterval_ms;
    if (!ok) {
        std::cout<<"NS3::Examples::"<<exampleName()
                <<"::setup: setting inter packet arrival fallback value: "
               << _interPacketInterval_ms <<std::endl;
        interPacketInterval_ms = _interPacketInterval_ms;
        ok = true;
    }
    uint16_t packetSize_B = parser.isSet(packetSizeOption) ?
                parser.value(packetSizeOption).toInt(&ok) :
                _packetSize_B;
    if (!ok) {
        std::cout<<"NS3::Examples::"<<exampleName()
                <<"::setup: setting packet size fallback value: "
               << _packetSize_B <<std::endl;
        packetSize_B = _packetSize_B;
        ok = true;
    }
    uint16_t runCount = parser.isSet(runCountOption) ?
                parser.value(runCountOption).toInt(&ok) :
                0;
    if (!ok) {
        std::cout<<"NS3::Examples::"<<exampleName()
                <<"::setup: setting run count fallback value: 0"<<std::endl;
        runCount = _runCount;
        ok = true;
    }

    Simulation::getInstance()->setName(exampleName() + "_" +
                                       "t" + std::to_string(simTime_s)+ "_" +
                                       "ue" + std::to_string(numOfPairs) + "_"+
                                       "ipi"+ std::to_string(interPacketInterval_ms) + "_" +
                                       "ps" + std::to_string(packetSize_B))->setRunCount(runCount);
    std::cout << "Scenario params: " <<std::endl;
    std::cout << "- sim time: " << simTime_s << " seconds" << std::endl;
    std::cout << "- num of UEs: " << numOfPairs << std::endl;
    std::cout << "- inter packet interval: " << interPacketInterval_ms << " Milliseconds"<< std::endl;
    std::cout << "- packet size: " << packetSize_B << " Bytes" << std::endl;

    Time simTime = Seconds (simTime_s);
    Time interPacketInterval = MilliSeconds(interPacketInterval_ms);
    uint64_t packetSize = packetSize_B; // in Bytes - ns3 max is 63KB
    bool buildings = true;
    bool disablePl = false;
    bool showProgress = true;
    uint64_t packetCount = 1000000;

    g_numOfPairs = numOfPairs;


    srand(static_cast<unsigned int>(time(nullptr)));
    uint seed = static_cast<uint>(rand());
    RngSeedManager::SetSeed(seed);
    RngSeedManager::SetRun(static_cast<uint>(runCount));

    NodeContainer uavNodes, carNodes, allNodes;
    uavNodes.Create(numOfPairs);
    carNodes.Create(numOfPairs);
    allNodes.Add(uavNodes);
    allNodes.Add(carNodes);


    NS_LOG_INFO ("Installing WiFi and Internet stack.");
    std::string phyMode ("OfdmRate6MbpsBW10MHz");
//    WifiHelper wifi;
    WifiMacHelper wifiMac;

    wifiMac.SetType ("ns3::AdhocWifiMac");
    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
//    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper();
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
//    wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel");
    wifiChannel.AddPropagationLoss("ns3::DeterministicObstacleShadowingPropagationLossModel");
    wifiPhy.SetChannel (wifiChannel.Create ());
    // ns-3 supports generate a pcap trace
    wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);
    NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
    Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
    wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                           "DataMode",StringValue (phyMode),
                                           "ControlMode",StringValue (phyMode));
//    wifi80211p.EnableLogComponents();
    NetDeviceContainer nodeDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, allNodes);

    // Internet stack
    InternetStackHelper internet;
    internet.Install (allNodes);
    Ipv4AddressHelper ipAddrs;
    ipAddrs.SetBase ("192.168.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = ipAddrs.Assign (nodeDevices);

    // Setup mobility
    MobilityHelper mobilityHelper;
    std::vector<std::string> colors { "red", "blue", "yellow",
                                      "magenta", "cyan"};
    uint utilCount = colors.size();

    // Setup Mobility
    std::vector<VehicleProperties> vehiclesProps = generateCarProps(numOfPairs, colors);
    std::vector<VehicleProperties> UAVProps = generateUAVProps(numOfPairs, colors);
    vehiclesProps.insert(vehiclesProps.begin(), UAVProps.begin(), UAVProps.end());
    setupNS3MobilityFromVehicleProps(mobilityHelper, vehiclesProps, allNodes);

    if (buildings) {
        BuildingsHelper::Install (allNodes);
        BuildingsHelper::MakeMobilityModelConsistent();
    }

    uint16_t dlPort = 1100;
    uint16_t ulPort = 2000;
    uint16_t dlPort1 = 1101;
    uint16_t ulPort1 = 2001;
    uint16_t otherPort = 3000;
    ApplicationContainer clientApps;
    ApplicationContainer serverApps;
    for (uint32_t u = 0; u < allNodes.GetN (); ++u)
    {
        // UE(uav) <-> UE(car)
        if (!disablePl)
        {

            uint32_t receivingNodeIdx = 0;
            uint16_t port = 0;

            // store the node id -> ip association globally
            nodeIdToIp[
                    std::to_string(allNodes.Get(u)->GetId())
                    ] = interfaces.GetAddress (u);
//            if((u >= 3 && u < 5) || (u > 7)) continue;
            if (u < numOfPairs) {
                // current Node is UAV
                receivingNodeIdx = u + numOfPairs;
                // store the nodeId -> destination ip association globally
                nodeIdToDestIp[
                        std::to_string(allNodes.Get(u)->GetId())
                        ] = interfaces.GetAddress(receivingNodeIdx);
                port = dlPort1;
            } else {
                // current Node is Car
                // store the nodeId -> destination ip association globally
                receivingNodeIdx = u - numOfPairs;
                nodeIdToDestIp[
                        std::to_string(allNodes.Get(u)->GetId())
                        ] = interfaces.GetAddress(receivingNodeIdx);
                continue;
//                port = dlPort1;
            }

            // install Udpserver app on node receiving node
            PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), port));
            serverApps.Add (packetSinkHelper.Install (allNodes.Get(receivingNodeIdx)));

            // store the nodeId -> destination ip association globally
            // move it up to store the association even if node is not sending
//            nodeIdToDestIp[
//                    std::to_string(allNodes.Get(u)->GetId())
//                    ] = interfaces.GetAddress(receivingNodeIdx);

            // install udp client app on corresponding sending node
            std::cout << "nodeID: " << allNodes.Get(u)->GetId() << std::endl;
            std::cout << "setting up udp client to send to :"
                      <<nodeIdToDestIp.at(std::to_string(allNodes.Get(u)->GetId()))
                      <<std::endl;
            UdpClientHelper client (interfaces.GetAddress (receivingNodeIdx), port);
            client.SetAttribute ("Interval", TimeValue (interPacketInterval));
            client.SetAttribute ("MaxPackets", UintegerValue (packetCount));
            client.SetAttribute ("PacketSize", UintegerValue(packetSize));
            clientApps.Add (client.Install (allNodes.Get (u)));
        }
    }

    // Display simulator time progress
    if (showProgress)
    {
        uint32_t status_period = 1;
        Simulator::Schedule(Seconds(1),&Callbacks::PrintProgress, status_period);
    }

    serverApps.Start (MilliSeconds (10));
    clientApps.Start (MilliSeconds (40));

    wifiStats = CreateObject<Modules::NetStats::WifiNetStatsCalculator>();
    wifiStats->SetOutputFilename("../results/WifiStats_" +
                                 Simulation::getInstance()->getName() + "_" +
                                 std::to_string(runCount) + ".txt");


    // MacTx
//    Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx",
//       MakeCallback (&SendPacketCallback));
    // MacRx
//    Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx",
//       MakeCallback (&ReceivePacketCallback));
    // Packet sink Rx
//    Config::Connect ("/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx",
//                 MakeCallback(&ReceivePacketInSink));
//    Config::Connect ("/NodeList/*/ApplicationList/*/$ns3::PacketSink/RxWithAddresses",
//                 MakeCallback(&ReceivePacketInSinkWithAddresses));
    // Ipv4 Tx/Rx
    Config::Connect("/NodeList/*/$ns3::Ipv4L3Protocol/Tx",
                    MakeCallback(&SendPacketIpv4L3));
    Config::Connect("/NodeList/*/$ns3::Ipv4L3Protocol/Rx",
                    MakeCallback(&ReceivePacketIpv4L3));

    VehicleManager::getInstance()->enableMobilityBroadcastHelper();

    // Simulation setup end

    if (monitorPerformance()) {
        PerformanceMonitor::getInstance()->reset();
        PerformanceMonitor::getInstance()->setupWithExport("../results/Performance_" +
                                                           Simulation::getInstance()->getName() + "_" +
                                                           std::to_string(_runCount) +
                                                           ".csv");
    }

    Simulator::Stop (simTime);
    setupIpv4TrafficAnimation();
    wireUpLIMoSimAndStartSimulation();

}

} // namespace WaveFollower
} // namespace Examples
} // namespace NS3
} // namespace LIMoSim
