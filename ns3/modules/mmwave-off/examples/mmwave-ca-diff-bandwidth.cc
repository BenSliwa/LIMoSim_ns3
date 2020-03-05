/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store.h"
#include "ns3/mmwave-helper.h"
#include <ns3/buildings-helper.h>
#include "ns3/global-route-manager.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/log.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"
#include "ns3/point-to-point-helper.h"
#include <map>

using namespace ns3;
using namespace mmwave;

/* In this example, a single UE is connected with a single MmWave BS. The
 * UE is randomly placed inside a circle with radius of 150m and it moves with
 * random velocity between 2m/s and 4m/s. The BS is placed in the center with
 * height 15m. The system bandwidth is fixed at 1GHz. If CA is enabled, 2 CCs
 * are used and the total bandwidth is divided among the two according to
 * the bandDiv parameter, which is equal to the ratio between the badwidth
 * allocated to CC0 and the bandwidth allocated to CC1.
 */

void
TxMacPacketTraceUe (Ptr<OutputStreamWrapper> stream, uint16_t rnti, uint8_t ccId, uint32_t size)
{
  *stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << (uint32_t)ccId << '\t' << size << std::endl;
}

void
Traces (std::string filePath)
{
  std::string path = "/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/MmWaveUeMac/TxMacPacketTraceUe";
  filePath = filePath + "TxMacPacketTraceUe.txt";
  AsciiTraceHelper asciiTraceHelper;
  Ptr<OutputStreamWrapper> stream1 = asciiTraceHelper.CreateFileStream (filePath);
  *stream1->GetStream () << "Time" << "\t" << "CC" << '\t' << "Packet size" << std::endl;
  Config::ConnectWithoutContext (path, MakeBoundCallback (&TxMacPacketTraceUe, stream1));
}


int
main (int argc, char *argv[])
{
  bool useCa = true;
  uint32_t bandDiv = 2; // if useCa=true, CC0 get (1-1/bandDiv)*totalBandwidth, CC1 get (1/bandDiv)*totalBandwidth
  bool useRR = false;
  bool useEpc = false;
  bool blockage0 = false;
  bool blockage1 = false;
  int numRefSc0 = 864;
  int numRefSc1 = 864;
  uint32_t chunkPerRb0 = 72;
  uint32_t chunkPerRb1 = 72;
  double frequency0 = 28e9;
  double frequency1 = 73e9;
  double simTime = 5;
  uint32_t runSet = 1;
  std::string filePath;

  CommandLine cmd;
  cmd.AddValue ("useCa", "If enabled use 2 CC", useCa);
  cmd.AddValue ("bandDiv", "Bandwidth division factor (B1=B0/bandDiv)", bandDiv);
  cmd.AddValue ("useRR", "If true use MmWaveRrComponentCarrierManager, else use MmWaveBaRrComponentCarrierManager", useRR);
  cmd.AddValue ("useEpc", "If enabled use EPC, else use RLC saturation mode", useEpc);
  cmd.AddValue ("blockage0", "If enabled CC0 blockage = true", blockage0);
  cmd.AddValue ("blockage1", "If enabled CC1 blockage = true", blockage1);
  cmd.AddValue ("frequency0", "CC0 central frequency", frequency0);
  cmd.AddValue ("frequency1", "CC1 central frequency", frequency1);
  cmd.AddValue ("simTime", "Simulation time", simTime);
  cmd.AddValue ("filePath", "Where to put the output files", filePath);
  cmd.AddValue ("runSet", "Run number", runSet);
  cmd.Parse (argc, argv);

  if (useCa)
    {
      chunkPerRb0 = chunkPerRb0 * (bandDiv - 1) / bandDiv;
      chunkPerRb1 = chunkPerRb1 / bandDiv;
      numRefSc0 = numRefSc0 * (bandDiv - 1) / bandDiv;
      numRefSc1 = numRefSc1 / bandDiv;
    }

  // RNG
  uint32_t seedSet = 1;
  RngSeedManager::SetSeed (seedSet);
  RngSeedManager::SetRun (runSet);

  // set output file names
  std::cout << "File path: " << filePath << std::endl;
  Config::SetDefault ("ns3::MmWaveBearerStatsCalculator::DlRlcOutputFilename", StringValue (filePath + "DlRlcStats.txt"));
  Config::SetDefault ("ns3::MmWaveBearerStatsCalculator::UlRlcOutputFilename", StringValue (filePath + "UlRlcStats.txt"));
  Config::SetDefault ("ns3::MmWaveBearerStatsCalculator::DlPdcpOutputFilename", StringValue (filePath + "DlPdcpStats.txt"));
  Config::SetDefault ("ns3::MmWaveBearerStatsCalculator::UlPdcpOutputFilename", StringValue (filePath + "UlPdcpStats.txt"));
  Config::SetDefault ("ns3::MmWavePhyRxTrace::OutputFilename", StringValue (filePath + "RxPacketTrace.txt"));
  Config::SetDefault ("ns3::LteRlcAm::BufferSizeFilename", StringValue (filePath + "RlcAmBufferSize.txt"));

  // CC 0
  // 1. create MmWavePhyMacCommon object
  Config::SetDefault ("ns3::MmWavePhyMacCommon::CenterFreq",DoubleValue (frequency0));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::ComponentCarrierId", UintegerValue (0));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::ChunkPerRB", UintegerValue (chunkPerRb0));
  Ptr<MmWavePhyMacCommon> phyMacConfig0 = CreateObject<MmWavePhyMacCommon> ();
  phyMacConfig0->SetNumRefScPerSym ( numRefSc0 );

  // 2. create the MmWaveComponentCarrier object
  Ptr<MmWaveComponentCarrier> cc0 = CreateObject<MmWaveComponentCarrier> ();
  cc0->SetConfigurationParameters (phyMacConfig0);
  cc0->SetAsPrimary (true);

  // CC 1
  Ptr<MmWaveComponentCarrier> cc1;
  if (useCa)
    {
      // 1. create MmWavePhyMacCommon object
      Config::SetDefault ("ns3::MmWavePhyMacCommon::CenterFreq",DoubleValue (frequency1));
      Config::SetDefault ("ns3::MmWavePhyMacCommon::ComponentCarrierId", UintegerValue (1));
      Config::SetDefault ("ns3::MmWavePhyMacCommon::ChunkPerRB", UintegerValue (chunkPerRb1));
      Ptr<MmWavePhyMacCommon> phyMacConfig1 = CreateObject<MmWavePhyMacCommon> ();
      phyMacConfig1->SetNumRefScPerSym ( numRefSc1 );

      // 2. create the MmWaveComponentCarrier object
      cc1 = CreateObject<MmWaveComponentCarrier> ();
      cc1->SetConfigurationParameters (phyMacConfig1);
      cc1->SetAsPrimary (false);
    }

  // create the CC map
  std::map<uint8_t, MmWaveComponentCarrier> ccMap;
  ccMap [0] = *cc0;
  if (useCa)
    {
      ccMap [1] = *cc1;
    }

  // set the blockage map
  std::map <uint8_t, bool> blockageMap;
  blockageMap [0] = blockage0;
  if (useCa)
    {
      blockageMap [1] = blockage1;
    }

  // print CC parameters
  for (uint8_t i = 0; i < ccMap.size (); i++)
    {
      Ptr<MmWavePhyMacCommon> phyMacConfig = ccMap[i].GetConfigurationParameters ();
      std::cout << "Component Carrier " << (uint32_t)(phyMacConfig->GetCcId ())
                << " frequency : " << phyMacConfig->GetCenterFrequency () / 1e9 << " GHz,"
                << " bandwidth : " << (phyMacConfig->GetNumRb () * phyMacConfig->GetChunkWidth () * phyMacConfig->GetNumChunkPerRb ()) / 1e6 << " MHz,"
                << " blockage : " << blockageMap[i]
                << ", numRefSc: " << (uint32_t)phyMacConfig->GetNumRefScPerSym ()
                << ", ChunkPerRB: " << phyMacConfig->GetNumChunkPerRb ()
                << std::endl;
    }


  // create and set the helper
  // first set UseCa = true, then NumberOfComponentCarriers
  Config::SetDefault ("ns3::MmWaveHelper::UseCa",BooleanValue (useCa));
  if (useCa)
    {
      Config::SetDefault ("ns3::MmWaveHelper::NumberOfComponentCarriers",UintegerValue (2));
      if (useRR)
        {
          Config::SetDefault ("ns3::MmWaveHelper::EnbComponentCarrierManager",StringValue ("ns3::MmWaveRrComponentCarrierManager"));
        }
      else
        {
          Config::SetDefault ("ns3::MmWaveHelper::EnbComponentCarrierManager",StringValue ("ns3::MmWaveBaRrComponentCarrierManager"));
        }
    }
  Config::SetDefault ("ns3::MmWaveHelper::ChannelModel",StringValue ("ns3::MmWave3gppChannel"));
  Config::SetDefault ("ns3::MmWaveHelper::PathlossModel",StringValue ("ns3::MmWave3gppPropagationLossModel"));

  // The available channel scenarios are 'RMa', 'UMa', 'UMi-StreetCanyon', 'InH-OfficeMixed', 'InH-OfficeOpen', 'InH-ShoppingMall'
  std::string scenario = "UMa";
  std::string condition = "n"; // n = NLOS, l = LOS
  Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::ChannelCondition", StringValue (condition));
  Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::Scenario", StringValue (scenario));
  Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::OptionalNlos", BooleanValue (false));
  Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::Shadowing", BooleanValue (false)); // enable or disable the shadowing effect
  Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::InCar", BooleanValue (false)); // enable or disable the shadowing effect

  Config::SetDefault ("ns3::MmWave3gppChannel::UpdatePeriod", TimeValue (MilliSeconds (100))); // Set channel update period, 0 stands for no update.
  Config::SetDefault ("ns3::MmWave3gppChannel::DirectBeam", BooleanValue (true)); // Set true to perform the beam in the exact direction of receiver node.
  Config::SetDefault ("ns3::MmWave3gppChannel::PortraitMode", BooleanValue (true)); // use blockage model with UT in portrait mode
  Config::SetDefault ("ns3::MmWave3gppChannel::NumNonselfBlocking", IntegerValue (4)); // number of non-self blocking obstacles
  Config::SetDefault ("ns3::MmWave3gppChannel::BlockerSpeed", DoubleValue (1)); // speed of non-self blocking obstacles

  Ptr<MmWaveHelper> helper = CreateObject<MmWaveHelper> ();
  helper->SetCcPhyParams (ccMap);
  helper->SetBlockageMap (blockageMap);

  // create the EPC
  Ipv4Address remoteHostAddr;
  Ptr<Node> remoteHost;
  InternetStackHelper internet;
  Ptr<MmWavePointToPointEpcHelper> epcHelper;
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  if (useEpc)
    {
      epcHelper = CreateObject<MmWavePointToPointEpcHelper> ();
      helper->SetEpcHelper (epcHelper);

      // create the Internet by connecting remoteHost to pgw. Setup routing too
      Ptr<Node> pgw = epcHelper->GetPgwNode ();

      // create remotehost
      NodeContainer remoteHostContainer;
      remoteHostContainer.Create (1);
      internet.Install (remoteHostContainer);
      Ipv4StaticRoutingHelper ipv4RoutingHelper;
      Ipv4InterfaceContainer internetIpIfaces;

      remoteHost = remoteHostContainer.Get (0);
      // create the Internet
      PointToPointHelper p2ph;
      p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
      p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
      p2ph.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (0.01)));

      NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);

      Ipv4AddressHelper ipv4h;
      ipv4h.SetBase ("1.0.0.0", "255.255.0.0");
      internetIpIfaces = ipv4h.Assign (internetDevices);
      // interface 0 is localhost, 1 is the p2p device
      remoteHostAddr = internetIpIfaces.GetAddress (1);

      Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
      remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.255.0.0"), 1);
    }


  // create the enb node
  NodeContainer enbNodes;
  enbNodes.Create (1);

  // set mobility
  Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator> ();
  enbPositionAlloc->Add (Vector (0.0, 0.0, 15.0));

  MobilityHelper enbmobility;
  enbmobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  enbmobility.SetPositionAllocator (enbPositionAlloc);
  enbmobility.Install (enbNodes);
  BuildingsHelper::Install (enbNodes);

  // install enb device
  NetDeviceContainer enbNetDevices = helper->InstallEnbDevice (enbNodes);
  std::cout << "eNB device installed" << std::endl;

  // create ue node
  NodeContainer ueNodes;
  ueNodes.Create (1);

  // set mobility
  MobilityHelper uemobility;
  uemobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                               "Bounds", RectangleValue (Rectangle (-200, 200, -200, 200)));
  Config::SetDefault ("ns3::UniformDiscPositionAllocator::rho", DoubleValue (150));
  Config::SetDefault ("ns3::UniformDiscPositionAllocator::Z", DoubleValue (1.6));
  Ptr<UniformDiscPositionAllocator> uePositionAlloc = CreateObject<UniformDiscPositionAllocator> ();
  uemobility.SetPositionAllocator (uePositionAlloc);
  uemobility.Install (ueNodes.Get (0));
  BuildingsHelper::Install (ueNodes);
  std::cout << "UE initial position :" << ueNodes.Get (0)->GetObject<MobilityModel> ()->GetPosition () << std::endl;

  // install ue device
  NetDeviceContainer ueNetDevices = helper->InstallUeDevice (ueNodes);
  std::cout << "UE device installed" << std::endl;

  if (useEpc)
    {
      // install the IP stack on the UEs
      internet.Install (ueNodes);
      Ipv4InterfaceContainer ueIpIface;
      ueIpIface = epcHelper->AssignUeIpv4Address (ueNetDevices);
      // assign IP address to UEs, and install applications
      // set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNodes.Get (0)->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

      helper->AttachToClosestEnb (ueNetDevices, enbNetDevices);

      // install and start applications on UEs and remote host
      uint16_t dlPort = 1234;
      uint16_t ulPort = 2000;
      ApplicationContainer clientApps;
      ApplicationContainer serverApps;

      uint16_t interPacketInterval = 100;

      PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dlPort));
      PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
      serverApps.Add (dlPacketSinkHelper.Install (ueNodes.Get (0)));
      serverApps.Add (ulPacketSinkHelper.Install (remoteHost));

      UdpClientHelper dlClient (ueIpIface.GetAddress (0), dlPort);
      dlClient.SetAttribute ("Interval", TimeValue (MilliSeconds (interPacketInterval)));
      dlClient.SetAttribute ("MaxPackets", UintegerValue (1000000));

      UdpClientHelper ulClient (remoteHostAddr, ulPort);
      ulClient.SetAttribute ("Interval", TimeValue (MilliSeconds (interPacketInterval)));
      ulClient.SetAttribute ("MaxPackets", UintegerValue (1000000));

      clientApps.Add (dlClient.Install (remoteHost));
      clientApps.Add (ulClient.Install (ueNodes.Get (0)));

      serverApps.Start (Seconds (0.01));
      clientApps.Start (Seconds (0.01));
    }
  else
    {
      helper->AttachToClosestEnb (ueNetDevices, enbNetDevices);

      // activate a data radio bearer
      enum EpsBearer::Qci q = EpsBearer::GBR_CONV_VOICE;
      EpsBearer bearer (q);
      helper->ActivateDataRadioBearer (ueNetDevices, bearer);
    }

  helper->EnableTraces ();
  Traces (filePath); // enable UL MAC traces

  BuildingsHelper::MakeMobilityModelConsistent ();

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
