/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Joe Kopena <tjkopena@cs.drexel.edu>
 *
 * These applications are used in the WiFi Distance Test experiment,
 * described and implemented in test02.cc.  That file should be in the
 * same place as this file.  The applications have two very simple
 * jobs, they just generate and receive packets.  We could use the
 * standard Application classes included in the NS-3 distribution.
 * These have been written just to change the behavior a little, and
 * provide more examples.
 *
 */

#include "wifi-reach-mobile.h"

#include <ostream>
#include <sstream>

// ns3
#include <ns3/mobility-module.h>
#include <ns3/internet-module.h>
#include <ns3/stats-module.h>
#include <ns3/yans-wifi-helper.h>

// LIMoSim ns3
#include "ns3/limosimmobilitymodel.h"
#include "ns3/ns3setuphelpers.h"
#include "ns3/tags/timestamptag.h"
#include "ns3/applications/mobilityreceiverudp.h"
#include "ns3/applications/mobilitysenderudp.h"

// LIMoSim
#include "LIMoSim/world/vehiclemanager.h"
#include "LIMoSim/mobility/uav/reynolds/behaviors.h"



namespace LIMoSim {
namespace NS3 {
namespace Examples {

using namespace ns3;
using namespace NS3::Tags;

NS_LOG_COMPONENT_DEFINE ("WiFiReachMobile");

namespace WifiReachMobile {

Behavior * wanderBehavior() {
    return new Behavior_Wander();
}

Behavior *cohesionBehavior() {
    return new Behavior_Cohesion(1500);
}
void setup() {

    //------------------------------------------------------------
    //-- Create nodes and network stacks
    //--------------------------------------------
    NS_LOG_INFO ("Creating nodes.");
    NodeContainer LIMoSimNodes;
    LIMoSimNodes.Create (3);

    NS_LOG_INFO ("Installing WiFi and Internet stack.");
    WifiHelper wifi;
    WifiMacHelper wifiMac;
    wifiMac.SetType ("ns3::AdhocWifiMac");
    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
    wifiPhy.SetChannel (wifiChannel.Create ());
    NetDeviceContainer nodeDevices = wifi.Install (wifiPhy, wifiMac, LIMoSimNodes);

    InternetStackHelper internet;
    internet.Install (LIMoSimNodes);
    Ipv4AddressHelper ipAddrs;
    ipAddrs.SetBase ("192.168.0.0", "255.255.255.0");
    ipAddrs.Assign (nodeDevices);

    //------------------------------------------------------------
    //-- Setup physical layout
    //--------------------------------------------
    NS_LOG_INFO ("Installing LIMoSim Mobility");
    MobilityHelper mobility;

    std::vector<VehicleProperties> vehiclesProps = {
        VehicleProperties(
            Vector (200, 480, 50),
            LIMoSimUAV,
            "U0",
        VehicleUIProps("blue"),
        MakeCallback(&cohesionBehavior)
        ),
        VehicleProperties(
            Vector (250, 450, 50),
            LIMoSimUAV,
            "U1",
            VehicleUIProps("yellow"),
        MakeCallback(&wanderBehavior)
        ),
        VehicleProperties(
            Vector (230, 480, 50),
            LIMoSimUAV,
            "U2",
        VehicleUIProps("red"),
        MakeCallback(&wanderBehavior)
        )
    };

    setupNS3MobilityFromVehicleProps(mobility, vehiclesProps, LIMoSimNodes);


    //------------------------------------------------------------
    //-- Create a custom traffic source and sink
    //--------------------------------------------    
//    NS_LOG_INFO ("Create traffic source & sink.");
//    Ptr<ns3::Node> appSource2 = NodeList::GetNode (2);
//    Ptr<Sender> sender2 = CreateObject<Sender>();
//    appSource2->AddApplication (sender2);
//    sender2->SetStartTime (MilliSeconds (1000));

//    NS_LOG_INFO ("Create traffic source & sink.");
//    Ptr<ns3::Node> appSource = NodeList::GetNode (1);
//    Ptr<Sender> sender = CreateObject<Sender>();
//    appSource->AddApplication (sender);
//    sender->SetStartTime (MilliSeconds (1500));

//    Ptr<ns3::Node> appSink = NodeList::GetNode (0);
//    Ptr<Receiver> receiver = CreateObject<Receiver>();
//    appSink->AddApplication (receiver);
//    receiver->SetStartTime (Seconds (0));

//    Config::Set ("/NodeList/*/ApplicationList/*/$Sender/Destination",
//                 Ipv4AddressValue ("192.168.0.1"));

    NS_LOG_INFO ("Create traffic source & sink.");
    Ptr<ns3::Node> appSource2 = NodeList::GetNode (2);
    Ptr<Applications::MobilitySenderUDP> sender2 = CreateObject<Applications::MobilitySenderUDP>();
    appSource2->AddApplication (sender2);
    sender2->SetStartTime (MilliSeconds (1000));

    NS_LOG_INFO ("Create traffic source & sink.");
    Ptr<ns3::Node> appSource = NodeList::GetNode (1);
    Ptr<Applications::MobilitySenderUDP> sender = CreateObject<Applications::MobilitySenderUDP>();
    appSource->AddApplication (sender);
    sender->SetStartTime (MilliSeconds (1500));

    Ptr<ns3::Node> appSink = NodeList::GetNode (0);
    Ptr<Applications::MobilityReceiverUDP> receiver = CreateObject<Applications::MobilityReceiverUDP>();
    appSink->AddApplication (receiver);
    receiver->SetStartTime (Seconds (0));



    Config::Set ("/NodeList/*/ApplicationList/*/$MobilitySenderUDP/Destination",
                 Ipv4AddressValue ("192.168.0.1"));

    setupIpv4TrafficAnimation();

    //------------------------------------------------------------
    //-- Setup stats and data collection
    //--------------------------------------------

    // Create a DataCollector object to hold information about this run.
//    DataCollector data;
//    data.DescribeRun (experiment,
//                      strategy,
//                      input,
//                      runID);

    // Add any information we wish to record about this run.
//    data.AddMetadata ("author", "tjkopena");


    // Create a counter to track how many frames are generated.  Updates
    // are triggered by the trace signal generated by the WiFi MAC model
    // object.  Here we connect the counter to the signal via the simple
    // TxCallback() glue function defined above.
//    Ptr<CounterCalculator<uint32_t> > totalTx =
//      CreateObject<CounterCalculator<uint32_t> >();
//    totalTx->SetKey ("wifi-tx-frames");
//    totalTx->SetContext ("node[0]");
//    Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx",
//                     MakeBoundCallback (&TxCallback, totalTx));
//    data.AddDataCalculator (totalTx);

    // This is similar, but creates a counter to track how many frames
    // are received.  Instead of our own glue function, this uses a
    // method of an adapter class to connect a counter directly to the
    // trace signal generated by the WiFi MAC.
//    Ptr<PacketCounterCalculator> totalRx =
//      CreateObject<PacketCounterCalculator>();
//    totalRx->SetKey ("wifi-rx-frames");
//    totalRx->SetContext ("node[1]");
//    Config::Connect ("/NodeList/1/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx",
//                     MakeCallback (&PacketCounterCalculator::PacketUpdate,
//                                   totalRx));
//    data.AddDataCalculator (totalRx);




    // This counter tracks how many packets---as opposed to frames---are
    // generated.  This is connected directly to a trace signal provided
    // by our Sender class.
//    Ptr<PacketCounterCalculator> appTx =
//      CreateObject<PacketCounterCalculator>();
//    appTx->SetKey ("sender-tx-packets");
//    appTx->SetContext ("node[0]");
//    Config::Connect ("/NodeList/0/ApplicationList/*/$Sender/Tx",
//                     MakeCallback (&PacketCounterCalculator::PacketUpdate,
//                                   appTx));
//    data.AddDataCalculator (appTx);

    // Here a counter for received packets is directly manipulated by
    // one of the custom objects in our simulation, the Receiver
    // Application.  The Receiver object is given a pointer to the
    // counter and calls its Update() method whenever a packet arrives.
//    Ptr<CounterCalculator<> > appRx =
//      CreateObject<CounterCalculator<> >();
//    appRx->SetKey ("receiver-rx-packets");
//    appRx->SetContext ("node[1]");
//    receiver->SetCounter (appRx);
//    data.AddDataCalculator (appRx);




    // This DataCalculator connects directly to the transmit trace
    // provided by our Sender Application.  It records some basic
    // statistics about the sizes of the packets received (min, max,
    // avg, total # bytes), although in this scenaro they're fixed.
//    Ptr<PacketSizeMinMaxAvgTotalCalculator> appTxPkts =
//      CreateObject<PacketSizeMinMaxAvgTotalCalculator>();
//    appTxPkts->SetKey ("tx-pkt-size");
//    appTxPkts->SetContext ("node[0]");
//    Config::Connect ("/NodeList/0/ApplicationList/*/$Sender/Tx",
//                     MakeCallback
//                       (&PacketSizeMinMaxAvgTotalCalculator::PacketUpdate,
//                       appTxPkts));
//    data.AddDataCalculator (appTxPkts);


    // Here we directly manipulate another DataCollector tracking min,
    // max, total, and average propagation delays.  Check out the Sender
    // and Receiver classes to see how packets are tagged with
    // timestamps to do this.
//    Ptr<TimeMinMaxAvgTotalCalculator> delayStat =
//      CreateObject<TimeMinMaxAvgTotalCalculator>();
//    delayStat->SetKey ("delay");
//    delayStat->SetContext (".");
//    receiver->SetDelayTracker (delayStat);
//    data.AddDataCalculator (delayStat);

    //------------------------------------------------------------
    //-- Generate statistics output.
    //--------------------------------------------

    // Pick an output writer based in the requested format.
//    Ptr<DataOutputInterface> output = 0;
//    if (format == "omnet") {
//        NS_LOG_INFO ("Creating omnet formatted data output.");
//        output = CreateObject<OmnetDataOutput>();
//      } else if (format == "db") {
//      #ifdef STATS_HAS_SQLITE3
//        NS_LOG_INFO ("Creating sqlite formatted data output.");
//        output = CreateObject<SqliteDataOutput>();
//      #endif
//      } else {
//        NS_LOG_ERROR ("Unknown output format " << format);
//      }

//    // Finally, have that writer interrogate the DataCollector and save
//    // the results.
//    if (output != 0)
//      output->Output (data);


    wireUpLIMoSimAndStartSimulation();

}

void TxCallback (Ptr<CounterCalculator<uint32_t> > datac,
                 std::string path, Ptr<const Packet> packet) {
  NS_LOG_INFO ("Sent frame counted in " <<
               datac->GetKey ());
  datac->Update ();
  // end TxCallback
}

TypeId
Sender::GetTypeId (void)
{
  static TypeId tid = TypeId ("Sender")
    .SetParent<Application> ()
    .AddConstructor<Sender> ()
    .AddAttribute ("PacketSize", "The size of packets transmitted.",
                   UintegerValue (64),
                   MakeUintegerAccessor (&Sender::m_pktSize),
                   MakeUintegerChecker<uint32_t>(1))
    .AddAttribute ("Destination", "Target host address.",
                   Ipv4AddressValue ("255.255.255.255"),
                   MakeIpv4AddressAccessor (&Sender::m_destAddr),
                   MakeIpv4AddressChecker ())
    .AddAttribute ("Port", "Destination app port.",
                   UintegerValue (1603),
                   MakeUintegerAccessor (&Sender::m_destPort),
                   MakeUintegerChecker<uint32_t>())
    .AddAttribute ("NumPackets", "Total number of packets to send.",
                   UintegerValue (150),
                   MakeUintegerAccessor (&Sender::m_numPkts),
                   MakeUintegerChecker<uint32_t>(1))
    .AddAttribute ("Interval", "Delay between transmissions.",
                   StringValue ("ns3::ConstantRandomVariable[Constant=0.5]"),
                   MakePointerAccessor (&Sender::m_interval),
                   MakePointerChecker <RandomVariableStream>())
    .AddTraceSource ("Tx", "A new packet is created and is sent",
                     MakeTraceSourceAccessor (&Sender::m_txTrace),
                     "ns3::Packet::TracedCallback")
  ;
  return tid;
}


Sender::Sender()
{
  NS_LOG_FUNCTION_NOARGS ();
  m_interval = CreateObject<ConstantRandomVariable> ();
  m_socket = 0;
}

Sender::~Sender()
{
  NS_LOG_FUNCTION_NOARGS ();
}

void
Sender::DoDispose (void)
{
  NS_LOG_FUNCTION_NOARGS ();

  m_socket = 0;
  // chain up
  Application::DoDispose ();
}

void Sender::StartApplication ()
{
  NS_LOG_FUNCTION_NOARGS ();

  if (m_socket == 0) {
      Ptr<SocketFactory> socketFactory = GetNode ()->GetObject<SocketFactory>
          (UdpSocketFactory::GetTypeId ());
      m_socket = socketFactory->CreateSocket ();
      m_socket->Bind ();
    }

  m_count = 0;

  Simulator::Cancel (m_sendEvent);
  m_sendEvent = Simulator::ScheduleNow (&Sender::SendPacket, this);

  Ptr<LIMoSim::NS3::LimoSimMobilityModel> mobility = GetNode()->GetObject<LIMoSim::NS3::LimoSimMobilityModel>();
  m_vehicleId = mobility->GetVehicleId();
  UAV* uav = dynamic_cast<UAV*>(VehicleManager::getInstance()->getVehicle(m_vehicleId));
  if (uav) {
      //uav->setBehavior(new Behavior_RandomWalk());
      uav->setBehavior(new Behavior_CircleAround(Vector3d(250,500,50),70, m_vehicleId));
      std::cout << "Sender: updating behavior to CIRCLE_AROUND_POSITION (250,500,50)" << std::endl;
  }

  // end Sender::StartApplication
}

void Sender::StopApplication ()
{
  NS_LOG_FUNCTION_NOARGS ();
  Simulator::Cancel (m_sendEvent);
  // end Sender::StopApplication
}

void Sender::SendPacket ()
{
  // NS_LOG_FUNCTION_NOARGS ();
  NS_LOG_INFO ("Sending packet at " << Simulator::Now () << " to " <<
               m_destAddr);

  std::vector<uint8_t> myVector(m_vehicleId.begin(), m_vehicleId.end());
  uint8_t *p = &myVector[0];
  Ptr<Packet> packet = Create<Packet>(p, m_pktSize);

  TimestampTag timestamp;
  timestamp.SetTimestamp (Simulator::Now ());
  packet->AddByteTag (timestamp);

  // Could connect the socket since the address never changes; using SendTo
  // here simply because all of the standard apps do not.
  m_socket->SendTo (packet, 0, InetSocketAddress (m_destAddr, m_destPort));

  // transmission animation for the UI
  // UI::AnimationUtils::animateTransmission(m_vehicleId);

  // Report the event to the trace.
  m_txTrace (packet);

  if (m_count == m_numPkts -1) {
      std::cout << "Sender: sending last packet." <<std::endl;
  }

  if (++m_count < m_numPkts) {
      m_sendEvent = Simulator::Schedule (Seconds (m_interval->GetValue ()),
                                         &Sender::SendPacket, this);
  }

  // end Sender::SendPacket
}




//----------------------------------------------------------------------
//-- Receiver
//------------------------------------------------------
TypeId
Receiver::GetTypeId (void)
{
  static TypeId tid = TypeId ("Receiver")
    .SetParent<Application> ()
    .AddConstructor<Receiver> ()
    .AddAttribute ("Port", "Listening port.",
                   UintegerValue (1603),
                   MakeUintegerAccessor (&Receiver::m_port),
                   MakeUintegerChecker<uint32_t>())
  ;
  return tid;
}

Receiver::Receiver() :
  m_calc (0),
  m_delay (0),
  m_recoverCount(0)
{
  NS_LOG_FUNCTION_NOARGS ();
  m_socket = 0;
}

Receiver::~Receiver()
{
  NS_LOG_FUNCTION_NOARGS ();
}

void
Receiver::DoDispose (void)
{
  NS_LOG_FUNCTION_NOARGS ();

  m_socket = 0;
  // chain up
  Application::DoDispose ();
}

void Receiver::updateBehavior()
{
    std::cout << "Receiver: received no packets for 10s, updating behavior to CIRCLE_AROUND_VEHICLE (Sender)" << std::endl;
    Ptr<LIMoSim::NS3::LimoSimMobilityModel> mobility = GetNode()->GetObject<LIMoSim::NS3::LimoSimMobilityModel>();
    std::string vehicleId = mobility->GetVehicleId();
    using namespace LIMoSim;
    UAV* uav = dynamic_cast<UAV*>(VehicleManager::getInstance()->getVehicle(vehicleId));
    if (uav) {
        uav->setBehavior(new Behavior_CircleAround(Target("U0"), 40, vehicleId));
        m_recoverCount = 0;
    }
}

void
Receiver::StartApplication ()
{
  NS_LOG_FUNCTION_NOARGS ();

  if (m_socket == 0) {
      Ptr<SocketFactory> socketFactory = GetNode ()->GetObject<SocketFactory>
          (UdpSocketFactory::GetTypeId ());
      m_socket = socketFactory->CreateSocket ();
      InetSocketAddress local = 
        InetSocketAddress (Ipv4Address::GetAny (), m_port);
      m_socket->Bind (local);
    }

  m_socket->SetRecvCallback (MakeCallback (&Receiver::Receive, this));
  m_rxTimeoutEvent = Simulator::Schedule ( Seconds(10), &Receiver::updateBehavior, this);

  Ptr<LIMoSim::NS3::LimoSimMobilityModel> mobility = GetNode()->GetObject<LIMoSim::NS3::LimoSimMobilityModel>();
  m_vehicleId = mobility->GetVehicleId();
  UAV* uav = dynamic_cast<UAV*>(VehicleManager::getInstance()->getVehicle(m_vehicleId));
  if (uav) {
      uav->setBehavior(new Behavior_Flee(Target("U1")));
  }

  // end Receiver::StartApplication
}

void
Receiver::StopApplication ()
{
  NS_LOG_FUNCTION_NOARGS ();

  if (m_socket != 0) {
      m_socket->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
    }

  // end Receiver::StopApplication
}

void
Receiver::SetCounter (Ptr<CounterCalculator<> > calc)
{
  m_calc = calc;
  // end Receiver::SetCounter
}
void
Receiver::SetDelayTracker (Ptr<TimeMinMaxAvgTotalCalculator> delay)
{
  m_delay = delay;
  // end Receiver::SetDelayTracker
}

void
Receiver::Receive (Ptr<Socket> socket)
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
          std::cout << "Received " << packet->GetSize () << " bytes from " <<
                                 InetSocketAddress::ConvertFrom (from).GetIpv4 () << std::endl;
        }

      Simulator::Cancel(m_rxTimeoutEvent);
      m_rxTimeoutEvent = Simulator::Schedule ( Seconds(10), &Receiver::updateBehavior, this);

      TimestampTag timestamp;
      // Should never not be found since the sender is adding it, but
      // you never know.
      if (packet->FindFirstMatchingByteTag (timestamp)) {
          Time tx = timestamp.GetTimestamp ();

          if (m_delay != 0) {
              m_delay->Update (Simulator::Now () - tx);
              static int packetNum = 0;
              std::cout << "Receiver: received packet " << ++packetNum << " with delay: " << Simulator::Now () - tx << std::endl;
              NS_LOG_INFO("delay of received packet is: " << Simulator::Now () - tx);
            }
        }

      if (m_calc != 0) {
          m_calc->Update ();
        }

      m_recoverCount++;

      if (m_recoverCount > 15) {
          Ptr<LIMoSim::NS3::LimoSimMobilityModel> mobility = GetNode()->GetObject<LIMoSim::NS3::LimoSimMobilityModel>();
          std::string vehicleId = mobility->GetVehicleId();
          using namespace LIMoSim;
          UAV* uav = dynamic_cast<UAV*>(VehicleManager::getInstance()->getVehicle(vehicleId));
          if (uav && uav->getModel()->getBehaviorName() != "Flee") {
              std::cout << "Receiver: stayed long enough in range, updating behavior to EXPLORE_AND_FLEE_FROM (Sender)" << std::endl;
              uav->setBehavior(new Behavior_Flee(Target("U1")));
              m_recoverCount = 0;
          }
      }

      // end receiving packets
    }

  // end Receiver::Receive
}

}
}
}
}
