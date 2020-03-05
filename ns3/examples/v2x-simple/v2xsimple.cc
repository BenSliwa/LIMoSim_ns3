#include "v2xsimple.h"

// ns3
#include "ns3/lte-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/config-store.h"
#include "ns3/lte-hex-grid-enb-topology-helper.h"
#include <ns3/buildings-helper.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/spectrum-analyzer-helper.h>
#include <ns3/multi-model-spectrum-channel.h>
//#include "ns3/netanim-module.h"    //Changes have to be made
#include "ns3/ns2-mobility-helper.h"
#include <ns3/flow-monitor-module.h>

#include <cfloat>
#include <sstream>


// LIMoSim ns3
#include "ns3/callbacks.h"
#include "ns3/ns3setuphelpers.h"
#include "ns3/tags/timestamptag.h"
#include "ns3/ns3utils.h"

// LIMoSim
#include "LIMoSim/mobility/uav/reynolds/behaviors.h"
#include "LIMoSim/simulation/simulation.h"
#include "LIMoSim/simulation/performancemonitor.h"
#include "LIMoSim/world/vehiclemanager.h"

#include "ui/data/uidatamanager.h"

#include "standalone/qclidecorators.h"

#include <QCommandLineParser>
#include <QString>

namespace LIMoSim {
namespace NS3 {
namespace Examples {
namespace V2XSimple {

using namespace ns3;
using Node = ns3::Node;
using MobilityModel = ns3::MobilityModel;

using namespace ns3;

//NS_LOG_COMPONENT_DEFINE ("d2d_communication_mode_2");

static int total_count_rx=0;
static int total_count_tx=0;

static bool persistence=false;

static Time interval;

static uint32_t nbcars;

static int cam_length;


// Output
static std::string simtime = "log_simtime_v2x.csv";
static std::string data = "log_data_v2x.csv";
static std::string connections = "log_connections_v2x.csv";
static std::string positions = "log_positions_v2x.csv";


static Ptr<OutputStreamWrapper> log_connections;
static Ptr<OutputStreamWrapper> log_simtime;
static Ptr<OutputStreamWrapper> log_positions;
static Ptr<OutputStreamWrapper> log_data;

// Global variables
static uint32_t ctr_totRx = 0; 	// Counter for total received packets
static uint32_t ctr_totTx = 0; 	// Counter for total transmitted packets
static uint16_t lenCam;
static double baseline= 150.0;        // Baseline distance in meter (150m for urban, 320m for freeway)

// Responders users
static NodeContainer ueVeh;
static NodeContainer enbVeh;

static uint16_t nodeIdOffset = 2;

inline std::string exampleName() {
    return "V2XSimple";
}

Behavior* behaviorFactory(std::string _targetId) {
    return new Behavior_FollowAtElevation(_targetId,20.0,40.0,false);
//    return new Behavior_NoOp();
}

NS_LOG_COMPONENT_DEFINE (exampleName());

std::vector<VehicleProperties> generateCarProps(uint numberOfCars, std::vector<std::string> colors) {
    std::vector<VehicleProperties> vehiclesProps;
    for (uint i = 0; i < numberOfCars; i++) {
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
    for (uint i = 0; i < numberOfUAVs; i++) {
        UAVProps.push_back(
                    VehicleProperties(
                        Vector(i*50 + 400,300,60),
                        LIMoSimUAV,
                        "U" + std::to_string(i),
                        VehicleUIProps(colors.at(i%colors.size())),
                        MakeBoundCallback(&behaviorFactory, "C" + std::to_string(i))
                        )
                    );
    }
    return UAVProps;
}


void
connectGroupsInUi (std::vector < NetDeviceContainer > groups)
{
    auto uidm = UI::Data::UIDataManager::getInstance();

    std::vector < NetDeviceContainer >::iterator gIt;
    for (gIt = groups.begin() ; gIt != groups.end() ; gIt++)
    {
        for (uint32_t i = 1; i < (*gIt).GetN(); i++)
        {
            uidm->registerConnection(std::to_string((*gIt).Get (0)->GetNode()->GetId()),
                                     std::to_string((*gIt).Get (i)->GetNode()->GetId()));
        }
    }
}


void
PrintStatus (uint32_t s_period, Ptr<OutputStreamWrapper> log_simtime)
{
    if (ctr_totRx > ctr_totTx)
    {
        ctr_totRx = ctr_totTx;
    }
    *log_simtime->GetStream() << Simulator::Now ().GetSeconds ()
                              << ";" << ctr_totRx << ";" << ctr_totTx << ";"
                              << (double) ctr_totRx / ctr_totTx << std::endl;

    std::cout << Simulation::getInstance()->getName() << "_r"
              << Simulation::getInstance()->getRunCount()
              << " t=" <<  Simulator::Now().GetSeconds()
              << "\t Rx/Tx="<< ctr_totRx << "/" << ctr_totTx
              << "\t PRR=" << (double) ctr_totRx / ctr_totTx << std::endl;
    Simulator::Schedule(Seconds(s_period), &PrintStatus, s_period,log_simtime);
}

void
SidelinkV2xAnnouncementMacTrace(Ptr<Socket> socket)
{
    Ptr <Node> node = socket->GetNode();
    uint32_t id = node->GetId();
    uint32_t simTime = Simulator::Now().GetSeconds();
    Ptr<MobilityModel> posMobility = node->GetObject<MobilityModel>();
    Vector posTx = posMobility->GetPosition();

    // Get receiver node
    Ptr<Node> rcvNode;
    if (id >= nbcars + nodeIdOffset) {
        //current node is Car -> rcv is UAV
        // cars do not send
        // rcvNode = enbVeh.Get(id - nbcars - nodeIdOffset);
        return;
    } else {
        // current node is UAV -> rcv is Car
       rcvNode = ueVeh.Get(id - nodeIdOffset);
    }

    // check for distance to transmitter
    Ptr<MobilityModel> mob = rcvNode->GetObject<MobilityModel>();
    Vector posRx = mob->GetPosition();

    double distance = sqrt(pow((posTx.x - posRx.x),2.0)+pow((posTx.y - posRx.y),2.0));
    if  (distance > 0 && distance <= baseline)
    {
        ctr_totTx++;
        UI::AnimationUtils::animateTransmission(std::to_string(node->GetId()));
    }

    // Generate CAM
    std::ostringstream msgCam;
    uint16_t packetSize = lenCam;

    msgCam << id-1 << "\t"
           << simTime << "\t"
           << ctr_totTx << "\t"
           << (int) posTx.x<< "\t"
           << (int) posTx.y << '\0';
    Ptr<Packet> packet = Create<Packet>((uint8_t*)msgCam.str().c_str(),packetSize);
    Tags::TimestampTag timestamp;
    timestamp.SetTimestamp (Simulator::Now());
    packet->AddByteTag(timestamp);
    socket->Send(packet);
}

static void
ReceivePacket(Ptr<Socket> socket)
{
    Ptr<Node> node = socket->GetNode();
    Ptr<MobilityModel> posMobility = node->GetObject<MobilityModel>();
    Vector posRx = posMobility->GetPosition();
    Ptr<Packet> packet = socket->Recv ();
    uint8_t *buffer = new uint8_t[packet->GetSize()];
    packet->CopyData(buffer,packet->GetSize());
    std::string s = std::string((char*)buffer);

    size_t pos = 0;
    std::string copy = s;
    std::string token;
    int posTx_x;
    int posTx_y;
    for (int i = 0; i < 4; i++)
    {
        if (copy.find("\t") != std::string::npos)
        {
            pos = copy.find("\t");
            token = copy.substr(0,pos);
            if(i == 3)
            {
                posTx_x = atoi(token.c_str());
            }
            copy.erase (0,pos+1);
        }
    }
    posTx_y = atoi(copy.c_str());


    UI::AnimationUtils::animateReception(std::to_string(node->GetId()));
    uint32_t id = node->GetId();
    double simTime = Simulator::Now().GetSeconds();
    ctr_totRx++;
    total_count_rx++;

    Tags::TimestampTag timestamp;
    double delay_ms = 0;
    if (packet->FindFirstMatchingByteTag(timestamp)){
        Time tx = timestamp.GetTimestamp();
        delay_ms = (Simulator::Now() - tx).GetSeconds();
    } else {
        std::cout << "no timestamp for delay computation" << std::endl;
    }

    double pdr = ctr_totRx * 1.0 / ctr_totTx;
    double datarate = ctr_totRx * packet->GetSize() * 8 / ( 1024*1024*Simulator::Now().GetSeconds());
    *log_data->GetStream() << ctr_totRx << "\t"
                           << simTime << "\t"
                           << id-1 << "\t"
                           << s << "\t"
                           << packet->GetSize() * 1000 << "\t"
                           << delay_ms << "\t"
                           << pdr << "\t"
                           << datarate << std::endl;

    std::cout << ctr_totRx << "\t"
                           << simTime << "\t"
                           << id-1 << "\t"
                           << s << "\t"
                           << packet->GetSize() * 1000 << "\t"
                           << delay_ms << "\t"
                           << pdr << "\t"
                           << datarate << std::endl;
}


void setup(int _runCount, uint16_t _numOfCars, uint16_t _simTime_s)
{
    LogComponentEnable (exampleName().c_str(), LOG_INFO);
    NS_LOG_INFO("setting up ns3 simulation setup of " << exampleName());
    std::cout << "setting up ns3 simulation setup of "
              << exampleName()
              << " run count:"
              << _runCount
              <<std::endl;

    QCommandLineParser parser;
    parser.setApplicationDescription("LIMoSim - ns3 - V2XFollower");
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
    QCommandLineOption runCountOption(
                QStringList() << "r" << "run-count",
                "Run count",
                "number");

    parser.addOptions({
                          simTimeOption,
                          ueCountOption,
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

    // Set cli sim params
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
                _numOfCars;
    if (!ok) {
        std::cout<<"NS3::Examples::"<<exampleName()
                <<"::setup: setting ue count fallback value: "
               << _numOfCars <<std::endl;
        numOfPairs = _numOfCars;
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
                                       "nc" + std::to_string(numOfPairs))
            ->setRunCount(runCount);
    std::cout << "Scenario params: " <<std::endl;
    std::cout << "- sim time: " << simTime_s << " seconds" << std::endl;
    std::cout << "- num of Pairs: " << numOfPairs << std::endl;
    std::cout << "- inter packet interval (fixed): " << 10 << " milliseconds"<< std::endl;
    std::cout << "- packet size (fixed): " << 190 << " Bytes" << std::endl;

    PerformanceMonitor::getInstance()->reset();
    PerformanceMonitor::getInstance()->setupWithExport("../results/Performance_" +
                                                       Simulation::getInstance()->getName() + "_" +
                                                       std::to_string(runCount) +
                                                       ".csv");
    setUAVNextToCar(true);

    // Initialize some values
    uint16_t simTime = simTime_s;                 // Simulation time in seconds
    uint32_t numVeh = numOfPairs;                  // Number of vehicles
    lenCam = 190;                           // Length of CAM message in bytes [50-300 Bytes]
    double ueTxPower = 23.0;                // Transmission power in dBm
    double probResourceKeep = 0.0;          // Probability to select the previous resource again [0.0-0.8]
    uint32_t mcs = 20;                      // Modulation and Coding Scheme
    bool harqEnabled = false;               // Retransmission enabled
    bool adjacencyPscchPssch = true;        // Subchannelization scheme
    bool partialSensing = false;            // Partial sensing enabled (actual only partialSensing is false supported)
    uint16_t sizeSubchannel = 10;           // Number of RBs per subchannel
    uint16_t numSubchannel = 3;             // Number of subchannels per subframe
    uint16_t startRbSubchannel = 0;         // Index of first RB corresponding to subchannelization
    uint16_t pRsvp = 100;				    // Resource reservation interval
    uint16_t t1 = 4;                        // T1 value of selection window
    uint16_t t2 = 100;                      // T2 value of selection window
    uint16_t slBandwidth;                   // Sidelink bandwidth
    std::string tracefile;                  // Name of the tracefile
    bool buildings = true;
    bool verbose = true;            // Print time progress

    ctr_totRx = 0;
    ctr_totTx = 0;
    total_count_rx = 0;
    total_count_tx = 0;

    srand(static_cast<unsigned int>(time(nullptr)));
    uint seed = static_cast<uint>(rand());
    RngSeedManager::SetSeed(seed);
    RngSeedManager::SetRun(static_cast<uint>(runCount));

    // Command line arguments
    CommandLine cmd;
    cmd.AddValue ("time", "Simulation Time", simTime);
    cmd.AddValue ("numVeh", "Number of Vehicles", numVeh);
    cmd.AddValue ("adjacencyPscchPssch", "Scheme for subchannelization", adjacencyPscchPssch);
    cmd.AddValue ("sizeSubchannel", "Number of RBs per Subchannel", sizeSubchannel);
    cmd.AddValue ("numSubchannel", "Number of Subchannels", numSubchannel);
    cmd.AddValue ("startRbSubchannel", "Index of first subchannel index", startRbSubchannel);
    cmd.AddValue ("T1", "T1 Value of Selection Window", t1);
    cmd.AddValue ("T2", "T2 Value of Selection Window", t2);
    //cmd.AddValue ("harqEnabled", "HARQ Retransmission Enabled", harqEnabled);
    //cmd.AddValue ("partialSensingEnabled", "Partial Sensing Enabled", partialSensing);
    cmd.AddValue ("lenCam", "Packetsize in Bytes", lenCam);
    cmd.AddValue ("mcs", "Modulation and Coding Scheme", mcs);
    cmd.AddValue ("pRsvp", "Resource Reservation Interval", pRsvp);
    cmd.AddValue ("probResourceKeep", "Probability for selecting previous resource again", probResourceKeep);
    cmd.AddValue ("log_simtime", "name of the simtime logfile", simtime);
    cmd.AddValue ("log_data", "name of the data logfile", data);
    cmd.AddValue ("tracefile", "Path of ns-3 tracefile", tracefile);
    cmd.AddValue ("baseline", "Distance in which messages are transmitted and must be received", baseline);
//    cmd.Parse(argc, argv);

    AsciiTraceHelper ascii;
    data = "../results/Pl_" +
            Simulation::getInstance()->getName() + "_" +
            std::to_string(runCount) +
            ".txt";
    log_simtime = ascii.CreateFileStream(simtime);
    log_data = ascii.CreateFileStream(data);
    log_connections = ascii.CreateFileStream(connections);
    log_positions = ascii.CreateFileStream(positions);
    nbcars = numVeh;

    NS_LOG_INFO ("Starting network configuration...");

    // Set the UEs power in dBm
    Config::SetDefault ("ns3::LteUePhy::TxPower", DoubleValue (ueTxPower));
    Config::SetDefault ("ns3::LteUePhy::RsrpUeMeasThreshold", DoubleValue (-10.0));
    // Enable V2X communication on PHY layer
    Config::SetDefault ("ns3::LteUePhy::EnableV2x", BooleanValue (true));

    // Set power
    Config::SetDefault ("ns3::LteUePowerControl::Pcmax", DoubleValue (ueTxPower));
    Config::SetDefault ("ns3::LteUePowerControl::PsschTxPower", DoubleValue (ueTxPower));
    Config::SetDefault ("ns3::LteUePowerControl::PscchTxPower", DoubleValue (ueTxPower));

    if (adjacencyPscchPssch)
    {
        slBandwidth = sizeSubchannel * numSubchannel;
    }
    else
    {
        slBandwidth = (sizeSubchannel+2) * numSubchannel;
    }

    // Configure for UE selected
    Config::SetDefault ("ns3::LteUeMac::UlBandwidth", UintegerValue(slBandwidth));
    Config::SetDefault ("ns3::LteUeMac::EnableV2xHarq", BooleanValue(harqEnabled));
    Config::SetDefault ("ns3::LteUeMac::EnableAdjacencyPscchPssch", BooleanValue(adjacencyPscchPssch));
    Config::SetDefault ("ns3::LteUeMac::EnablePartialSensing", BooleanValue(partialSensing));
    Config::SetDefault ("ns3::LteUeMac::SlGrantMcs", UintegerValue(mcs));
    Config::SetDefault ("ns3::LteUeMac::SlSubchannelSize", UintegerValue (sizeSubchannel));
    Config::SetDefault ("ns3::LteUeMac::SlSubchannelNum", UintegerValue (numSubchannel));
    Config::SetDefault ("ns3::LteUeMac::SlStartRbSubchannel", UintegerValue (startRbSubchannel));
    Config::SetDefault ("ns3::LteUeMac::SlPrsvp", UintegerValue(pRsvp));
    Config::SetDefault ("ns3::LteUeMac::SlProbResourceKeep", DoubleValue(probResourceKeep));
    Config::SetDefault ("ns3::LteUeMac::SelectionWindowT1", UintegerValue(t1));
    Config::SetDefault ("ns3::LteUeMac::SelectionWindowT2", UintegerValue(t2));
//    Config::SetDefault ("ns3::LteUeMac::EnableExcludeSubframe", BooleanValue(excludeSubframe));



    // Create node container to hold all UEs
    NodeContainer ueAllNodes;
    NodeContainer enbAllNodes;

//    if (!tracefile.empty())
//    {
//        // Create nodes
//        ueVeh.Create (numVeh);
//        ueAllNodes.Add (ueVeh);

//        Ns2MobilityHelper ns2 = Ns2MobilityHelper(tracefile);
//        ns2.Install();
//    }

    NS_LOG_INFO ("Creating helpers...");
    // EPC
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    Ptr<Node> pgw = epcHelper->GetPgwNode();

    // LTE Helper
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    lteHelper->SetEpcHelper(epcHelper);
    lteHelper->DisableNewEnbPhy(); // Disable eNBs for out-of-coverage modelling


    // V2X
    Ptr<LteV2xHelper> lteV2xHelper = CreateObject<LteV2xHelper> ();
    lteV2xHelper->SetLteHelper (lteHelper);

    // Configure eNBs' antenna parameters before deploying them.
    lteHelper->SetEnbAntennaModelType ("ns3::NistParabolic3dAntennaModel");

    lteHelper->SetAttribute ("UseSameUlDlPropagationCondition", BooleanValue(true));
    Config::SetDefault ("ns3::LteEnbNetDevice::UlEarfcn", StringValue ("54990"));
    // Set pathloss model
    if (buildings) {
        lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::HybridBuildingsPropagationLossModel"));
        lteHelper->SetPathlossModelAttribute ("ShadowSigmaOutdoor", DoubleValue (0.0));
        lteHelper->SetPathlossModelAttribute ("ShadowSigmaIndoor", DoubleValue (0.0));
        lteHelper->SetPathlossModelAttribute ("ShadowSigmaExtWalls", DoubleValue (0.0));
    } else {
        lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::DeterministicObstacleShadowingSpectrumPropagationLossModel"));
//        lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::FriisSpectrumPropagationLossModel"));
    }
//    Config::SetDefault ("ns3::CniUrbanmicrocellPropagationLossModel::Frequency", DoubleValue(5.9e9));
//    lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::CniUrbanmicrocellPropagationLossModel"));

    // Create eNB Container
    NodeContainer eNodeB;
    eNodeB.Create(1);

    // Topology eNodeB
    Ptr<ListPositionAllocator> pos_eNB = CreateObject<ListPositionAllocator>();
    pos_eNB->Add(Vector(5,-10,30));

    //  Install mobility eNodeB
    MobilityHelper mob_eNB;
    mob_eNB.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob_eNB.SetPositionAllocator(pos_eNB);
    mob_eNB.Install(eNodeB);

    // Install Service
    NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice(eNodeB);

    // Create nodes
    ueVeh = NodeContainer();
    enbVeh = NodeContainer();
    enbVeh.Create (numVeh);
    ueVeh.Create (numVeh);
    ueAllNodes.Add(enbVeh);
    ueAllNodes.Add (ueVeh);
    std::cout << "nodeid: " << enbVeh.Get(0)->GetId() << std::endl;

    int utilcount = 5;
    std::vector<std::string> colors { "cyan", "magenta", "blue",
                                      "yellow", "red"};

    MobilityHelper mobilityHelper;

//    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
//    for (uint16_t i = 0; i < ueAllNodes.GetN(); i++)
//    {
//        positionAlloc->Add (Vector(i*50, 0, 150)); // ENBs positions
//    }

//    mobilityHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
//    mobilityHelper.SetPositionAllocator(positionAlloc);
//    mobilityHelper.Install(ueAllNodes);

    std::vector<VehicleProperties> vehiclesProps = generateUAVProps(enbVeh.GetN(), colors);
    setupNS3MobilityFromVehicleProps(mobilityHelper, vehiclesProps, enbVeh);
    vehiclesProps = generateCarProps(ueVeh.GetN(), colors);
    setupNS3MobilityFromVehicleProps(mobilityHelper, vehiclesProps, ueVeh);

    std::vector<NodeContainer> nodePairs;
    for (uint16_t i = 0; i < numOfPairs; i++) {
        NodeContainer pair;
        pair.Add(enbVeh.Get(i));
        pair.Add(ueVeh.Get(i));
        nodePairs.push_back(pair);
    }


    // Required to use NIST 3GPP model
    BuildingsHelper::Install (eNodeB);
    BuildingsHelper::Install (ueAllNodes);
    BuildingsHelper::MakeMobilityModelConsistent ();

    // Install LTE devices to all UEs
    NS_LOG_INFO ("Installing UE's network devices ...");
    lteHelper->SetAttribute ("UseSidelink", BooleanValue (true));
    std::vector<NetDeviceContainer> ueRespDevPairs;
    NetDeviceContainer ueDevs;
    for (uint16_t i = 0; i < numOfPairs; i++) {
        NetDeviceContainer ueRespondersDevsPair = lteHelper->InstallUeDevice(nodePairs.at(i));
        ueDevs.Add(ueRespondersDevsPair);
        // quick hack
//        ueRespondersDevsPair.Add(ueRespondersDevsPair.Get(1));
        ueRespDevPairs.push_back(ueRespondersDevsPair);
    }

//    NetDeviceContainer ueRespondersDevs = lteHelper->InstallUeDevice(ueAllNodes);
//    NetDeviceContainer ueDevs;
//    ueDevs.Add (ueRespondersDevs);

    // Install the IP stack on the UEs
    NS_LOG_INFO ("Installing IP stack...");
    InternetStackHelper internet;
    internet.Install (ueAllNodes);

    // Assign IP address to UEs
    NS_LOG_INFO ("Allocating IP addresses and setting up network route...");
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address (ueDevs);
    Ipv4StaticRoutingHelper ipv4RoutingHelper;

    for (uint32_t u = 0; u < ueAllNodes.GetN (); ++u)
      {
        Ptr<Node> ueNode = ueAllNodes.Get (u);
        // Set the default gateway for the UE
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
        ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
      }

    NS_LOG_INFO ("Attaching UE's to LTE network...");
    // Attach each UE to the best available eNB
    lteHelper->Attach (ueDevs);


    NS_LOG_INFO ("Creating sidelink groups...");
    // Create groups
    std::vector<std::vector<NetDeviceContainer>> txGroupsPairs;
    for (uint16_t i = 0; i < numOfPairs; i++) {
        // quick hack consequence: fictional third member in pair.
        std::vector<NetDeviceContainer> txGroups = lteV2xHelper->AssociateForV2xBroadcast(ueRespDevPairs.at(i), 2);
//        std::vector<NetDeviceContainer> txGroups = lteV2xHelper->AssociateForV2xBroadcast(ueRespDevPairs.at(i), 3);
        txGroupsPairs.push_back(txGroups);
        std::cout << "Sidelink group #" << i << std::endl;
        lteV2xHelper->PrintGroups(txGroups);
    }
//    std::vector<NetDeviceContainer> txGroups;
//    txGroups = lteV2xHelper->AssociateForV2xBroadcast(ueRespondersDevs, numVeh);
//    lteV2xHelper->PrintGroups(txGroups);


    //compute average number of receivers associated per transmitter and vice versa
    double totalRxs = 0;
    std::map < uint32_t, uint32_t> txPerUeMap;
    std::map < uint32_t, uint32_t> groupsPerUe;
    // Application Setup for Responders
    std::vector<uint32_t> groupL2Addresses;
    uint32_t groupL2Address = 0x00;
    Ipv4AddressGenerator::Init(Ipv4Address ("225.0.0.0"), Ipv4Mask("255.0.0.0"));
    Ipv4Address clientRespondersAddress = Ipv4AddressGenerator::NextAddress (Ipv4Mask ("255.0.0.0"));

    for (uint16_t i = 0; i < numOfPairs; i++) {
        std::vector<NetDeviceContainer> txGroups = txGroupsPairs.at(i);
        connectGroupsInUi(txGroups);
        std::vector < NetDeviceContainer >::iterator gIt;
        for (gIt = txGroups.begin() ; gIt != txGroups.end() ; gIt++)
        {
            uint32_t numDevs = gIt->GetN ();

              totalRxs += numDevs - 1;
              uint32_t nId;

            for (uint32_t i = 1; i < numDevs; i++)
            {
                  nId = gIt->Get (i)->GetNode ()->GetId ();
                  txPerUeMap[nId]++;
            }
        }

        double totalTxPerUe = 0;
        std::map < uint32_t, uint32_t>::iterator mIt;
    //    std::cout << "Tx per UE:" << std::endl;
        for (mIt = txPerUeMap.begin (); mIt != txPerUeMap.end() ; mIt++)
          {
            totalTxPerUe+= mIt->second;
    //        std::cout << mIt->first << " " << mIt->second << std::endl;
            groupsPerUe [mIt->second]++;
          }
        lteV2xHelper->PrintGroups (txGroups, log_connections);
//        std::cout << "Associated Groups per Rx UE" << std::endl;
//        for (mIt = groupsPerUe.begin (); mIt != groupsPerUe.end (); mIt++)
//        {
//            std::cout << mIt->first << " " << mIt->second << std::endl;
//        }



        NS_LOG_INFO ("Installing applications...");

//        // Application Setup for Responders
//        std::vector<uint32_t> groupL2Addresses;
//        uint32_t groupL2Address = 0x00;
//        Ipv4AddressGenerator::Init(Ipv4Address ("225.0.0.0"), Ipv4Mask("255.0.0.0"));
//        Ipv4Address clientRespondersAddress = Ipv4AddressGenerator::NextAddress (Ipv4Mask ("255.0.0.0"));

        uint16_t application_port = 8000; // Application port to TX/RX
        NetDeviceContainer activeTxUes;


        for(gIt=txGroups.begin(); gIt != txGroups.end(); gIt++)
        {
            // Create Sidelink bearers
            // Use Tx for the group transmitter and Rx for all the receivers
            // Split Tx/Rx

            NetDeviceContainer txUe ((*gIt).Get(0));
            activeTxUes.Add(txUe);
            NetDeviceContainer rxUes = lteV2xHelper->RemoveNetDevice ((*gIt), txUe.Get (0));
            Ptr<LteSlTft> tft = Create<LteSlTft> (LteSlTft::TRANSMIT, clientRespondersAddress, groupL2Address);
            lteV2xHelper->ActivateSidelinkBearer (Seconds(0.0), txUe, tft);
            tft = Create<LteSlTft> (LteSlTft::RECEIVE, clientRespondersAddress, groupL2Address);
            lteV2xHelper->ActivateSidelinkBearer (Seconds(0.0), rxUes, tft);

            std::cout << "Created group L2Address=" << groupL2Address << " IPAddress=";
            clientRespondersAddress.Print(std::cout);
            std::cout << std::endl;

            //Individual Socket Traffic Broadcast everyone
            Ptr<Socket> host = Socket::CreateSocket(txUe.Get(0)->GetNode(),TypeId::LookupByName ("ns3::UdpSocketFactory"));
            host->Bind();
            host->Connect(InetSocketAddress(clientRespondersAddress,application_port));
            host->SetAllowBroadcast(true);
            host->ShutdownRecv();

            /*Ptr<LteUeRrc> ueRrc = DynamicCast<LteUeRrc>( txUe.Get (0)->GetObject<LteUeNetDevice> ()->GetRrc () );
            ueRrc->TraceConnectWithoutContext ("SidelinkV2xMonitoring", MakeBoundCallback (&SidelinkV2xMonitoringTrace, stream));
            oss << txUe.Get(0) ->GetObject<LteUeNetDevice>()->GetImsi();
            Ptr<LteUePhy> uePhy = DynamicCast<LteUePhy>( txUe.Get (0)->GetObject<LteUeNetDevice> ()->GetPhy () );
            uePhy->TraceConnect ("SidelinkV2xAnnouncement", oss.str() ,MakeBoundCallback (&SidelinkV2xAnnouncementPhyTrace, stream1));*/
            //uePhy->TraceConnectWithoutContext ("SidelinkV2xAnnouncement", MakeBoundCallback (&SidelinkV2xAnnouncementPhyTrace, host));
            Ptr<LteUeMac> ueMac = DynamicCast<LteUeMac>( txUe.Get (0)->GetObject<LteUeNetDevice> ()->GetMac () );
            ueMac->TraceConnectWithoutContext ("SidelinkV2xAnnouncement", MakeBoundCallback (&SidelinkV2xAnnouncementMacTrace, host));
            //ueMac->TraceConnect ("SidelinkV2xAnnouncement", oss.str() ,MakeBoundCallback (&SidelinkV2xAnnouncementMacTrace, stream2));

            Ptr<Socket> sink = Socket::CreateSocket(txUe.Get(0)->GetNode(),TypeId::LookupByName ("ns3::UdpSocketFactory"));
            sink->Bind(InetSocketAddress (Ipv4Address::GetAny (), application_port));
            sink->SetRecvCallback (MakeCallback (&ReceivePacket));

            //store and increment addresses
            groupL2Addresses.push_back (groupL2Address);
            groupL2Address++;
            clientRespondersAddress = Ipv4AddressGenerator::NextAddress (Ipv4Mask ("255.0.0.0"));
        }
    }

    NS_LOG_INFO ("Creating Sidelink Configuration...");
    Ptr<LteUeRrcSl> ueSidelinkConfiguration = CreateObject<LteUeRrcSl>();
    ueSidelinkConfiguration->SetSlEnabled(true);
    ueSidelinkConfiguration->SetV2xEnabled(true);

    LteRrcSap::SlV2xPreconfiguration preconfiguration;
    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommPreconfigGeneral.carrierFreq = 54890;
    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommPreconfigGeneral.slBandwidth = slBandwidth;

    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.nbPools = 1;
    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.nbPools = 1;

//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.pools[0].slSubframeV2x.bitmap = std::bitset<20> (0xFFFFF);
//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.pools[0].adjacencyPscchPssch.adjacency = adjacencyPscchPssch;
//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.pools[0].sizeSubchannel = LteRrcSap::SizeSubchannelFromInt (sizeSubchannel);
//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.pools[0].numSubchannel = LteRrcSap::NumSubchannelFromInt (numSubchannel);
//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.pools[0].startRbSubchannel.startRb = startRbSubchannel;
//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.pools[0].dataTxParameters.p0 = -4;
//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.pools[0].dataTxParameters.alpha = LteRrcSap::SlTxParameters::al09;

//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.nbPools = 1;
//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.pools[0].slSubframeV2x.bitmap = std::bitset<20> (0xFFFFF);
//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.pools[0].adjacencyPscchPssch.adjacency = adjacencyPscchPssch;
//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.pools[0].sizeSubchannel = LteRrcSap::SizeSubchannelFromInt (sizeSubchannel);
//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.pools[0].numSubchannel = LteRrcSap::NumSubchannelFromInt (numSubchannel);
//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.pools[0].startRbSubchannel.startRb = startRbSubchannel;
//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.pools[0].dataTxParameters.p0 = -4;
//    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.pools[0].dataTxParameters.alpha = LteRrcSap::SlTxParameters::al09;

    SlV2xPreconfigPoolFactory pFactory;
    pFactory.SetHaveUeSelectedResourceConfig (true);
    pFactory.SetSlSubframe (std::bitset<20> (0xFFFFF));
    pFactory.SetAdjacencyPscchPssch (adjacencyPscchPssch);
    pFactory.SetSizeSubchannel (sizeSubchannel);
    pFactory.SetNumSubchannel (numSubchannel);
    pFactory.SetStartRbSubchannel (startRbSubchannel);
    pFactory.SetStartRbPscchPool (0);
    pFactory.SetDataTxP0 (-4);
    pFactory.SetDataTxAlpha (0.9);
    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.pools[0] = pFactory.CreatePool ();
    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.pools[0] = pFactory.CreatePool ();
    ueSidelinkConfiguration->SetSlV2xPreconfiguration (preconfiguration);


    // Print Configuration
    *log_data->GetStream() << "RxPkts\tRxTime\tRxId\tTxId\tTxTime\tTxPkts\tPosX\tPosy\tBytes\tDelay\tPDR\tDatarate" << std::endl;

    NS_LOG_INFO ("Installing Sidelink Configuration...");
    lteHelper->InstallSidelinkV2xConfiguration (ueDevs, ueSidelinkConfiguration);

    NS_LOG_INFO ("Enabling LTE traces...");
//    lteHelper->EnableTraces();
//    Ptr<RadioBearerStatsCalculator> pdcpStats = lteHelper->GetPdcpStats ();
//    pdcpStats->SetDlPdcpOutputFilename("../results/DlPdcpStats_" + exampleName() + "_" + std::to_string(runCount) + ".txt");

    *log_simtime->GetStream() << "Simtime; Total Rx; Total Tx; PRR" << std::endl;
    // Display simulator time progress
    Simulator::Schedule(Seconds(1), &PrintStatus, 1, log_simtime);



    VehicleManager::getInstance()->enableMobilityBroadcastHelper();

//    Ptr<FlowMonitor> flowMonitor;
//    FlowMonitorHelper flowHelper;
//    flowMonitor = flowHelper.InstallAll();

    NS_LOG_INFO ("Starting simulation...");
    Simulator::Stop(MilliSeconds(simTime*1000+40));
//    Simulator::Run();
//    Simulator::Destroy();

    wireUpLIMoSimAndStartSimulation();


//     flowMonitor->SerializeToXmlFile("Flow_" +
//                                     exampleName() + "_" +
//                                     std::to_string(runCount) +
//                                     ".xml",
//                                     true,
//                                     true);

//    NS_LOG_INFO ("Done.");
//    return 0;

}



} // namespace V2XSimple
} // namespace Examples
} // namespace NS3
} // namespace LIMoSim
