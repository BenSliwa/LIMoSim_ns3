#include "ltesimplestatic.h"


#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/config-store.h"
#include <ns3/buildings-helper.h>

#include "ns3/ns3setuphelpers.h"

namespace LIMoSim {
namespace NS3 {
namespace Examples {

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("LteSimpleStatic");

namespace LteSimpleStatic {

void setup()
{

    NS_LOG_INFO("setting up ns3 simulation setup of " << "LteSimpleStatic");
    std::cout << "setting up ns3 simulation setup of " << "LteSimpleStatic" <<std::endl;

    Time simTime = Seconds (10);
    bool useCa = false;

//    CommandLine cmd;
//    cmd.AddValue ("simTime", "Total duration of the simulation", simTime);
//    cmd.AddValue ("useCa", "Whether to use carrier aggregation.", useCa);
//    cmd.Parse (argc, argv);

    // to save a template default attribute file run it like this:
    // ./waf --command-template="%s --ns3::ConfigStore::Filename=input-defaults.txt --ns3::ConfigStore::Mode=Save --ns3::ConfigStore::FileFormat=RawText" --run src/lte/examples/lena-simple
    //
    // to load a previously created default attribute file
    // ./waf --command-template="%s --ns3::ConfigStore::Filename=input-defaults.txt --ns3::ConfigStore::Mode=Load --ns3::ConfigStore::FileFormat=RawText" --run src/lte/examples/lena-simple

    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults ();

    // Parse again so you can override default values from the command line
//    cmd.Parse (argc, argv);

    if (useCa)
    {
     Config::SetDefault ("ns3::LteHelper::UseCa", BooleanValue (useCa));
     Config::SetDefault ("ns3::LteHelper::NumberOfComponentCarriers", UintegerValue (2));
     Config::SetDefault ("ns3::LteHelper::EnbComponentCarrierManager", StringValue ("ns3::RrComponentCarrierManager"));
    }

    Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();

    // Uncomment to enable logging
    //  lteHelper->EnableLogComponents ();

    // Create Nodes: eNodeB and UE
    NodeContainer enbNodes;
    NodeContainer ueNodes;
    enbNodes.Create (1);
    ueNodes.Create (1);

    // Install Mobility Model
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(250, 450, 50));
    positionAlloc->Add(Vector(200, 480, 50));
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (enbNodes);
    BuildingsHelper::Install (enbNodes);
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (ueNodes);
    BuildingsHelper::Install (ueNodes);

    // Create Devices and install them in the Nodes (eNB and UE)
    NetDeviceContainer enbDevs;
    NetDeviceContainer ueDevs;
    // Default scheduler is PF, uncomment to use RR
    //lteHelper->SetSchedulerType ("ns3::RrFfMacScheduler");

    enbDevs = lteHelper->InstallEnbDevice (enbNodes);
    ueDevs = lteHelper->InstallUeDevice (ueNodes);

    // Attach a UE to a eNB
    lteHelper->Attach (ueDevs, enbDevs.Get (0));

    // Activate a data radio bearer
    enum EpsBearer::Qci q = EpsBearer::GBR_CONV_VOICE;
    EpsBearer bearer (q);
    lteHelper->ActivateDataRadioBearer (ueDevs, bearer);
    lteHelper->EnableTraces ();

    Simulator::Stop (simTime);

    setupIpv4TrafficAnimation();
    setupLtePdcpTrafficAnimation();
    wireUpLIMoSimAndStartSimulation();
}

}

}
}
}
