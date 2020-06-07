#include "ltestaticfollower.h"

#include<ctime>

#include "ns3/core-module.h"
#include "ns3/lte-helper.h"
#include "ns3/lte-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include <ns3/buildings-helper.h>
#include <ns3/point-to-point-helper.h>
#include <ns3/config-store.h>

// LIMoSim ns3
#include "ns3/callbacks.h"
#include "ns3/ns3setuphelpers.h"
#include "ns3/limosimmobilitymodel.h"

// LIMoSim
#include "LIMoSim/mobility/uav/reynolds/behaviors.h"
#include "LIMoSim/mobility/car/car.h"
#include "LIMoSim/world/world.h"
#include "LIMoSim/world/vehiclemanager.h"
#include "LIMoSim/world/road/road.h"
#include "LIMoSim/world/road/lanesegment.h"
//#include "LIMoSim/world/road/lane.h"
#include "LIMoSim/world/road/roadsegment.h"
#include "LIMoSim/simulation/performancemonitor.h"
#include "LIMoSim/settings/filehandler.h"
#include "LIMoSim/utils/typedefs.h"
#include "LIMoSim/settings/osm/wgs84.h"
#include "ui/data/uidatamanager.h"

#include "standalone/qclidecorators.h"

#include <QCommandLineParser>
#include <QString>
#include <QStringList>

namespace LIMoSim {
namespace NS3 {
namespace Examples {
namespace LteStaticFollower {



using namespace ns3;
using Node = ns3::Node;
using namespace LIMoSim::utils::typedefs;


inline std::string exampleName() {
    return "LteStaticFollower";
}

NS_LOG_COMPONENT_DEFINE (exampleName());

std::vector<Vector3d> readBasestationsFile() {
    StringVector lines = LIMoSim::FileHandler::read("bs_tmobile_results.csv");

    // erase header line
    lines.erase(lines.begin());

    Vector3d origin = World::getInstance()->getReference();

    std::vector<Vector3d> positions;

    for (std::string line : lines) {
        QStringList parts = QString::fromStdString(line).split(',');
        if (parts.size() != 7 || parts.at(0).isEmpty()) {
            continue;
        }
        Vector3d position (parts.at(3).toDouble(), parts.at(2).toDouble());
        Vector3d cart = WGS84::computeOffset(position, origin);
        positions.push_back(cart); std::cout << cart << std::endl;
    }
    return positions;
}

//  UI Stuff for base station link


static std::map<uint64_t, uint16_t> imsiToCellId;

NetDeviceContainer* ueLteDevsPointer(NetDeviceContainer * _netdevs = nullptr) {
    static NetDeviceContainer * pointer = _netdevs;
    return pointer;
}

NetDeviceContainer* enbLteDevsPointer(NetDeviceContainer *_netdevs = nullptr) {
    static NetDeviceContainer * pointer = _netdevs;
    return pointer;
}

std::string nodeIdFromImsi(uint64_t _imsi, NetDeviceContainer& _ueLteDevs) {
    for (uint16_t i = 0; i < _ueLteDevs.GetN(); i++) {
        Ptr<LteUeNetDevice> netDev = DynamicCast<LteUeNetDevice>(_ueLteDevs.Get(i));
        if (_imsi == netDev->GetImsi()) {
            return std::to_string(netDev->GetNode()->GetId());
        }
    }
    return "";
}

std::string nodeIdFromCellId(uint64_t _cellId, NetDeviceContainer& _enbLteDevs) {
    for (uint16_t i = 0; i < _enbLteDevs.GetN(); i++) {
        Ptr<LteEnbNetDevice> netDev = DynamicCast<LteEnbNetDevice>(_enbLteDevs.Get(i));
        uint64_t cellId= netDev->GetCellId();
        if (_cellId == cellId) {
            return std::to_string(netDev->GetNode()->GetId());
        }
    }
    return "";
}

void
NotifyHandoverEndOkUe (std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti)
{
  std::cout << Simulator::Now ().GetSeconds () << " " << context
            << " UE IMSI " << imsi
            << ": successful handover to CellId " << cellid
            << " with RNTI " << rnti
            << std::endl;
  imsiToCellId[imsi] = cellid;
  auto uidm = UI::Data::UIDataManager::getInstance();
  std::string node1 = nodeIdFromCellId(cellid, *enbLteDevsPointer());
//  std::string _node1 = antagonsitNodeIdFromCellId(cellid, *enbLteDevsPointer());
  std::string node2 = nodeIdFromImsi(imsi, *ueLteDevsPointer());
  if (!node2.empty()) {
//      uidm->unregisterConnection(_node1, node2);
      uidm->unregisterConnectionsOf(node2);
      uidm->registerConnection(node1, node2);
  }
}

void
NotifyConnectionEstablishedEnb (std::string context,
                                uint64_t imsi,
                                uint16_t cellid,
                                uint16_t rnti)
{
  std::cout << Simulator::Now ().GetSeconds () << " " << context
            << " eNB CellId " << cellid
            << ": successful connection of UE with IMSI " << imsi
            << " RNTI " << rnti
            << std::endl;
    imsiToCellId[imsi] = cellid;
    auto uidm = UI::Data::UIDataManager::getInstance();
    std::string node1 = nodeIdFromCellId(cellid, *enbLteDevsPointer());
//    std::string _node1 = antagonsitNodeIdFromCellId(cellid, *enbLteDevsPointer());
    std::string node2 = nodeIdFromImsi(imsi, *ueLteDevsPointer());
    if (!node2.empty()) {
//        uidm->unregisterConnection(_node1, node2);
        uidm->unregisterConnectionsOf(node2);
        uidm->registerConnection(node1, node2);
    }
}

void setupUIConnections(NetDeviceContainer _ueLteDevs) {
    auto uidm = UI::Data::UIDataManager::getInstance();
//    uidm->clearConnections();
    for (uint16_t i = 0; i < _ueLteDevs.GetN(); i++) {
        Ptr<LteUeNetDevice> netDev = DynamicCast<LteUeNetDevice>(_ueLteDevs.Get(i));
        uint16_t cellId = imsiToCellId.at(netDev->GetImsi());
        std::string node1 = std::to_string(cellId - 1);
//        std::string node1 = cellId == 1 ? "0" : "1";
        uidm->registerConnection(node1, std::to_string(netDev->GetNode()->GetId()));
    }
}

// ///////////////////////////////////////////////////


//std::vector<VehicleProperties> generateCarProps(uint numberOfUes) {
//    std::vector<VehicleProperties> vehiclesProps;
//    for (uint i = 0; i < numberOfUes; i++) {
//        vehiclesProps.push_back(
//                    VehicleProperties(
//                        Vector(0,0,0),
//                        LIMoSimCar,
//                        "C" + std::to_string(i),
//                        VehicleUIProps("red")
//                        )
//                    );
//    }
//    return vehiclesProps;
//}
Behavior* behaviorFactory(std::string _targetId) {
//    return new Behavior_FollowAtElevation(_targetId);
    return new Behavior_FollowAtElevation(_targetId, 30,40,false);
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

void setup(int _runCount, uint16_t _simTime_s, uint16_t _numOfUes, uint16_t _interPacketInterval_ms, uint16_t _packetSize_B)
{
    LogComponentEnable(exampleName().c_str(), LOG_LEVEL_INFO);
    NS_LOG_INFO("setting up ns3 simulation setup of " << exampleName());
    std::cout << "setting up ns3 simulation setup of " << exampleName() << " run count:" << _runCount <<std::endl;

    setUAVNextToCar(true);

    auto enbPositions = readBasestationsFile();

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
                QStringList() << "ue" << "ue-count",
                "Number of user equipments",
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
//                          droneInteractionModeOption,
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
        std::cout<<"NS3::Examples::LteCoverageMobile::setup: setting simTime fallback value: " << _simTime_s <<std::endl;
        simTime_s = _simTime_s;
        ok = true;
    }
    uint16_t numOfPairs = parser.isSet(ueCountOption) ?
                parser.value(ueCountOption).toInt(&ok) :
                _numOfUes;
    if (!ok) {
        std::cout<<"NS3::Examples::LteCoverageMobile::setup: setting ue count fallback value: " << _numOfUes <<std::endl;
        numOfPairs = _numOfUes;
        ok = true;
    }
    uint16_t interPacketInterval_ms = parser.isSet(interPackterIntervalOption) ?
                parser.value(interPackterIntervalOption).toInt(&ok) :
                _interPacketInterval_ms;
    if (!ok) {
        std::cout<<"NS3::Examples::LteCoverageMobile::setup: setting inter packet arrival fallback value: " << _interPacketInterval_ms <<std::endl;
        interPacketInterval_ms = _interPacketInterval_ms;
        ok = true;
    }
    uint16_t packetSize_B = parser.isSet(packetSizeOption) ?
                parser.value(packetSizeOption).toInt(&ok) :
                _packetSize_B;
    if (!ok) {
        std::cout<<"NS3::Examples::LteCoverageMobile::setup: setting packet size fallback value: " << _packetSize_B <<std::endl;
        packetSize_B = _packetSize_B;
        ok = true;
    }
    uint16_t runCount = parser.isSet(runCountOption) ?
                parser.value(runCountOption).toInt(&ok) :
                0;
    if (!ok) {
        std::cout<<"NS3::Examples::ParcelDelivery::setup: setting run count fallback value: 0"<<std::endl;
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

    PerformanceMonitor::getInstance()->reset();
    PerformanceMonitor::getInstance()->setupWithExport("../results/Performance_" +
                                                       Simulation::getInstance()->getName() + "_" +
                                                       std::to_string(runCount) +
                                                       ".csv");

    uint16_t numOfEnbs = enbPositions.size();
    Time simTime = Seconds (simTime_s);
    Time interPacketInterval = MilliSeconds(interPacketInterval_ms);
    uint64_t packetSize = packetSize_B; // in Bytes - ns3 max is 63KB

    bool disableDl = true;
    bool disableUl = true;
    bool disablePl = false;
    bool showProgress = true;
    bool buildings = false;

    uint32_t dlEarfcn = 1300;
    uint32_t ulEarfcn = 19300;
    uint32_t dlResBlck = 100;
    uint32_t ulResBlck = 100;

    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults ();

    Config::SetDefault ("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue (false));
    Config::SetDefault ("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue (false));
    Config::SetDefault ("ns3::LteAmc::AmcModel", EnumValue (LteAmc::PiroEW2010));
    Config::SetDefault ("ns3::LteAmc::Ber", DoubleValue (0.00005));
    Config::SetDefault ("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue(ulResBlck));
    Config::SetDefault ("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue(dlResBlck));
    Config::SetDefault ("ns3::LteEnbNetDevice::UlEarfcn", UintegerValue(ulEarfcn));
    Config::SetDefault ("ns3::LteEnbNetDevice::DlEarfcn", UintegerValue(dlEarfcn));
    Config::SetDefault ("ns3::PhyStatsCalculator::DlRsrpSinrFilename",
                        StringValue("../results/DlRsrpSinrStats_" +
                                    Simulation::getInstance()->getName() + "_" +
                                    std::to_string(runCount) + ".txt"));
    Config::SetDefault ("ns3::PhyStatsCalculator::UlSinrFilename",
                        StringValue("../results/UlSinrStats_" +
                                    Simulation::getInstance()->getName() + "_" +
                                    std::to_string(runCount) + ".txt"));

    srand(static_cast<unsigned int>(time(nullptr)));
    uint seed = static_cast<uint>(rand());
    RngSeedManager::SetSeed (seed);
    RngSeedManager::SetRun (static_cast<uint>(runCount));

    Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
    if (buildings) {
        lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::HybridBuildingsPropagationLossModel"));
        lteHelper->SetPathlossModelAttribute ("ShadowSigmaOutdoor", DoubleValue (0.0));
        lteHelper->SetPathlossModelAttribute ("ShadowSigmaIndoor", DoubleValue (0.0));
        lteHelper->SetPathlossModelAttribute ("ShadowSigmaExtWalls", DoubleValue (0.0));
    } else {
        lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::DeterministicObstacleShadowingSpectrumPropagationLossModel"));
//        lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::FriisSpectrumPropagationLossModel"));
    }

    Config::SetDefault ("ns3::LteUePhy::EnableUplinkPowerControl", BooleanValue (false));
    lteHelper->SetSchedulerType ("ns3::RrFfMacScheduler");
    lteHelper->SetSchedulerAttribute ("UlCqiFilter", EnumValue (FfMacScheduler::PUSCH_UL_CQI));
    lteHelper->SetHandoverAlgorithmType ("ns3::A3RsrpHandoverAlgorithm");

    Ptr<PointToPointEpcHelper>  epcHelper = CreateObject<PointToPointEpcHelper> ();
    lteHelper->SetEpcHelper (epcHelper);

    Ptr<Node> pgw = epcHelper->GetPgwNode ();

    // Create a single RemoteHost
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create (1);
    Ptr<Node> remoteHost = remoteHostContainer.Get (0);
    InternetStackHelper internet;
    internet.Install (remoteHostContainer);

    // Create the Internet
    // IP Stack for remote host
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
    p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
    p2ph.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (10)));
    NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
    // interface 0 is localhost, 1 is the p2p device
    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
    remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);


    NodeContainer enbNodes, ueNodes, ueUAVNodes, ueCarNodes;
    enbNodes.Create(numOfEnbs);
    ueUAVNodes.Create(numOfPairs);
    ueCarNodes.Create(numOfPairs);
    ueNodes.Add(ueUAVNodes);
    ueNodes.Add(ueCarNodes);

    std::vector<std::string> colors { "red", "blue", "yellow",
                                      "magenta", "cyan"};
    uint utilCount = colors.size();

    // Setup Mobility
    std::vector<VehicleProperties> vehiclesProps = generateCarProps(numOfPairs, colors);
    std::vector<VehicleProperties> UAVProps = generateUAVProps(numOfPairs, colors);
    vehiclesProps.insert(vehiclesProps.begin(), UAVProps.begin(), UAVProps.end());

//    std::vector<VehicleProperties> vehiclesProps = generateUAVProps(numOfPairs, colors);
//    std::vector<VehicleProperties> CarProps = generateCarProps(numOfPairs, colors);
//    vehiclesProps.insert(vehiclesProps.begin(), CarProps.begin(), CarProps.end());
    MobilityHelper mobility;
    setupNS3MobilityFromVehicleProps(mobility, vehiclesProps, ueNodes);

    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
    for (uint16_t i = 0; i < numOfEnbs; i++)
    {
//        positionAlloc->Add (Vector(0, 0, 40)); // ENBs positions
        positionAlloc->Add (Vector(enbPositions.at(i).x, enbPositions.at(i).y, 60));
    }

    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(enbNodes);

    if (buildings) {
        BuildingsHelper::Install (enbNodes);
        BuildingsHelper::Install (ueNodes);
        BuildingsHelper::MakeMobilityModelConsistent();
    }

    // Install LTE Devices to the nodes
    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNodes);
    NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (ueNodes);

    enbLteDevsPointer(&enbLteDevs);
    ueLteDevsPointer(&ueLteDevs);


    // Install the IP stack on the UEs
    internet.Install (ueNodes);
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueLteDevs));
    // Assign IP address to UEs, and install applications
    for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
        Ptr<Node> ueNode = ueNodes.Get (u);
        // Set the default gateway for the UE
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
        ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

    // Attach all UEs to first eNodeB
//    for (uint16_t i = 0; i < numOfUes; i++)
//    {
//        lteHelper->Attach (ueLteDevs.Get(i), enbLteDevs.Get(0));
//    }
    // Attach each UE to the nearest eNodeB
    lteHelper->AttachToClosestEnb(ueLteDevs, enbLteDevs);

    // Install and start applications on UEs and remote host
    uint16_t dlPort = 1100;
    uint16_t ulPort = 2000;
    uint16_t dlPort1 = 1101;
    uint16_t ulPort1 = 2001;
    uint16_t otherPort = 3000;
    ApplicationContainer clientApps;
    ApplicationContainer serverApps;
    for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
        // Downlink remote host -> UE
        if (!disableDl)
        {
            PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dlPort));
            serverApps.Add (dlPacketSinkHelper.Install (ueNodes.Get(u)));

            UdpClientHelper dlClient (ueIpIface.GetAddress (u), dlPort);
            dlClient.SetAttribute ("Interval", TimeValue (interPacketInterval));
            dlClient.SetAttribute ("MaxPackets", UintegerValue (1000000));
            dlClient.SetAttribute ("PacketSize", UintegerValue(packetSize));
            clientApps.Add (dlClient.Install (remoteHost));
        }

        // Uplink remote host <- UE
        if (!disableUl)
        {
            ++ulPort;
            PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
            serverApps.Add (ulPacketSinkHelper.Install (remoteHost));

            UdpClientHelper ulClient (remoteHostAddr, ulPort);
            ulClient.SetAttribute ("Interval", TimeValue (interPacketInterval));
            ulClient.SetAttribute ("MaxPackets", UintegerValue (1000000));
            ulClient.SetAttribute ("PacketSize", UintegerValue(packetSize));
            clientApps.Add (ulClient.Install (ueNodes.Get(u)));
        }

        // UE(uav) -> UE(car)
        if (!disablePl)
        {

            uint32_t receivingNodeIdx = 0;
            uint16_t port = 0;
            if (u < numOfPairs) {
                // current Node is UAV
                receivingNodeIdx = u + numOfPairs;
                port = dlPort1;
            } else {
                // current Node is Car
                continue;
//                receivingNodeIdx = u- numOfPairs;
//                port = dlPort1;
            }

            // install Udpserver app on node receiving node
            PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), port));
            serverApps.Add (packetSinkHelper.Install (ueNodes.Get(receivingNodeIdx)));

            // install udp client app on corresponding sending node
            UdpClientHelper client (ueIpIface.GetAddress (receivingNodeIdx), port);
            client.SetAttribute ("Interval", TimeValue (interPacketInterval));
            client.SetAttribute ("MaxPackets", UintegerValue (1000000));
            client.SetAttribute ("PacketSize", UintegerValue(packetSize));
            clientApps.Add (client.Install (ueNodes.Get (u)));
        }
    }

    // Add X2 interface
    lteHelper->AddX2Interface (enbNodes);

    for (uint i = 0; i < enbLteDevs.GetN(); i++) {
        Ptr<LteEnbNetDevice> lteEnbDev = enbLteDevs.Get (i)->GetObject<LteEnbNetDevice> ();
        Ptr<LteEnbPhy> enbPhy = lteEnbDev->GetPhy ();
        enbPhy->SetAttribute ("TxPower", DoubleValue (43.0));
        enbPhy->SetAttribute ("NoiseFigure", DoubleValue (5.0));
    }

    // Set UEs' position and power
    for (uint i = 0; i < ueLteDevs.GetN(); i++)
    {
        Ptr<LteUeNetDevice> lteUeDev = ueLteDevs.Get (i)->GetObject<LteUeNetDevice> ();
        Ptr<LteUePhy> uePhy = lteUeDev->GetPhy ();
        uePhy->SetAttribute ("TxPower", DoubleValue (23.0));
        uePhy->SetAttribute ("NoiseFigure", DoubleValue (9.0));
    }

    // Display simulator time progress
    if (showProgress)
    {
        uint32_t status_period = 1;
        Simulator::Schedule(Seconds(1),&Callbacks::PrintProgress, status_period);
    }

    serverApps.Start (MilliSeconds (10));
    clientApps.Start (MilliSeconds (40));
    lteHelper->EnablePdcpTraces();
//    lteHelper->EnablePhyTraces();
    VehicleManager::getInstance()->enableMobilityBroadcastHelper();

    Ptr<RadioBearerStatsCalculator> pdcpStats = lteHelper->GetPdcpStats ();
    pdcpStats->SetDlPdcpOutputFilename("../results/DlPdcpStats_" +
                                       Simulation::getInstance()->getName() + "_" +
                                       std::to_string(runCount) + ".txt");
    pdcpStats->SetUlPdcpOutputFilename("../results/UlPdcpStats_" +
                                       Simulation::getInstance()->getName() + "_" +
                                       std::to_string(runCount) + ".txt");

    // connect custom trace sinks for RRC connection establishment and handover notification
   Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
                    MakeCallback (&NotifyConnectionEstablishedEnb));
//   Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
//                    MakeCallback (&NotifyConnectionEstablishedUe));
//   Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
//                    MakeCallback (&NotifyHandoverStartEnb));
//   Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
//                    MakeCallback (&NotifyHandoverStartUe));
//   Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
//                    MakeCallback (&NotifyHandoverEndOkEnb));
   Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
      MakeCallback (&NotifyHandoverEndOkUe));


    Simulator::Stop (simTime);
    wireUpLIMoSimAndStartSimulation();
}

} // namespace LteStaticFollower
} // namespace Examples
} // namespace NS3
} // namespace LIMoSim
