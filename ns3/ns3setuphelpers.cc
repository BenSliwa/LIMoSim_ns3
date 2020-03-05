#include "ns3setuphelpers.h"

#include <ns3/core-module.h>
#include <ns3/network-module.h>
#include <ns3/mobility-module.h>
#include <ns3/ipv4-header.h>

#include "ns3/tags/timestamptag.h"

#include "LIMoSim/world/vehiclemanager.h"

#include "LIMoSim/world/world.h"
#include "LIMoSim/world/road/road.h"
#include "LIMoSim/world/road/lanesegment.h"
#include "LIMoSim/world/road/lane.h"
#include "LIMoSim/world/road/roadsegment.h"
#include "LIMoSim/world/road/roadutils.h"
#include "LIMoSim/world/vehiclemanager.h"

#include "LIMoSim/mobility/car/car.h"
#include "LIMoSim/mobility/car/strategic/strategicmodel.h"
#include "LIMoSim/mobility/external/vehiclenoderegistry.h"

#include "ui/data/uidatamanager.h"
#include "ui/uimanagerservice.h"

namespace LIMoSim {

namespace NS3 {

using namespace ns3;
using namespace Tags;
using namespace UI::Data;

NS_LOG_COMPONENT_DEFINE ("NS3SetupHelpers");

bool enableAnimationLog = true;
bool enableCallbackLog = false;
bool uavNextToCar = false;

void setupNS3Globals()
{
//    GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));
    Time::SetResolution (Time::NS);

    LogComponentEnable("NS3SetupHelpers", LOG_LEVEL_INFO);
}

void setupNS3MobilityFromVehicleProps(
        MobilityHelper &_mobility,
        std::vector<VehicleProperties> _vehiclesProps,
        NodeContainer &_LIMoSimNodes
        ) {
    uint32_t numOfLIMoSimNodes = _LIMoSimNodes.GetN();

    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    for (VehicleProperties vehicleProps : _vehiclesProps) {
        positionAlloc->Add(vehicleProps.position);
    }
    _mobility.SetPositionAllocator(positionAlloc);

    if (_vehiclesProps.size() < _LIMoSimNodes.GetN()) {
        NS_LOG_ERROR("not enough vehiclesProperties to match LIMoSim nodes sizes");
        std::cerr << "mismatch between vehiclesProperties and LIMoSim nodes sizes" << std::endl;
        exit(101);
    }

    for (uint32_t nodeIdx = 0; nodeIdx < numOfLIMoSimNodes; nodeIdx++) {
        VehicleProperties vehicleProps = _vehiclesProps.at(nodeIdx);
        _mobility.SetMobilityModel("LIMoSim::NS3::LIMoSimMobility",
                                  "VehicleType", EnumValue(vehicleProps.type),
                                  "VehicleId", StringValue(vehicleProps.id),
                                   "ShapeColor", StringValue(vehicleProps.ui.color),
                                   "BehaviorFactory", CallbackValue(vehicleProps.behaviorFactory));
        _mobility.Install(_LIMoSimNodes.Get(nodeIdx));
    }
}

void createLIMoSimVehicles()
{
    std::cout << "creating LIMoSim vehicles" << std::endl;
    NS_LOG_INFO("creating LIMoSim vehicles");
    uint32_t numNodes = NodeList::GetNNodes();
    for (uint32_t nodeIdx = 0; nodeIdx < numNodes; nodeIdx++) {
        Ptr<ns3::MobilityModel> mobility = NodeList::GetNode(nodeIdx)->GetObject<ns3::MobilityModel>();
        if (
                !mobility ||
                mobility->GetInstanceTypeId().GetName() != "LIMoSim::NS3::LIMoSimMobility"
                ) {
            continue;
        }

        StringValue vehicleId;
        StringValue shapeColor;
        EnumValue vehicleType;
        mobility->GetAttribute("VehicleId", vehicleId);
        mobility->GetAttribute("ShapeColor", shapeColor);
        mobility->GetAttribute("VehicleType", vehicleType);

        NS_LOG_INFO("creating LIMoSim vehicle: " << vehicleType.Get() << " " << vehicleId.Get());
        std::cout << "creating LIMoSim vehicle: " << vehicleType.Get() << " " << vehicleId.Get() << std::endl;

        Vehicle *vehicle;
        VehicleManager *vehicleManager = VehicleManager::getInstance();
        switch (vehicleType.Get()) {
        case LIMoSimCar:
            vehicle = carSetup(vehicleManager->createCar(vehicleId.Get()), mobility);
            break;
        case LIMoSimUAV:
            vehicle = UAVSetup(vehicleManager->createUAV(vehicleId.Get()), mobility);
            break;
        }

        mobility->Initialize();

        if (!vehicle) {
            NS_LOG_ERROR("error while creating LIMoSim vehicle: " << vehicleType.Get() << " " << vehicleId.Get());
            std::cout << "error while creating LIMoSim vehicle: " << vehicleType.Get() << " " << vehicleId.Get() << std::endl;
            continue;
        }

        // register node -> vehicle mapping
        std::string nodeId = static_cast<std::ostringstream*>( &(std::ostringstream() << nodeIdx) )->str();
        Mobility::External::VehicleNodeRegistry::getInstance()->registerVehicleNode(nodeId, vehicleId.Get());

        // register ui data
        UIDataManager* ui = UIDataManager::getInstance();
        ui->getVehicleData(vehicle->getId())->getShape()->setColor(shapeColor.Get());

        //mobility->
    }
}

void registerStaticNodes()
{
    std::cout << "registering static nodes to LIMoSim UI" << std::endl;
    NS_LOG_INFO("registering static nodes to LIMoSim UI");
    uint32_t numNodes = NodeList::GetNNodes();
    for (uint32_t nodeIdx = 0; nodeIdx < numNodes; nodeIdx++) {
        Ptr<ns3::MobilityModel> mobility = NodeList::GetNode(nodeIdx)->GetObject<ns3::MobilityModel>();
        if (
                !mobility ||
                mobility->GetInstanceTypeId().GetName() != "ns3::ConstantPositionMobilityModel"
                ) {
            continue;
        }

        int nodeIdNum = NodeList::GetNode(nodeIdx)->GetId();
        std::string nodeId = static_cast<std::ostringstream*>( &(std::ostringstream() << nodeIdNum) )->str();
        Ptr<ns3::ConstantPositionMobilityModel> constmobility = NodeList::GetNode(nodeIdx)->GetObject<ns3::ConstantPositionMobilityModel>();
        if (!constmobility) {
            NS_LOG_ERROR("cannot retrieve constant mobility model on static node " << nodeId);
            std::cout << "cannot retrieve constant mobility model on static node " << nodeId << std::endl;
            continue;
        }
        UIDataManager::getInstance()->registerStaticNode(nodeId, toLIMoSimVector(constmobility->GetPosition()), Orientation3d());
    }
}



Vehicle* carSetup(Car *_car, Ptr<ns3::MobilityModel> _mobilityModel)
{
    RoadPosition roadPosition = RoadUtils::randomValidRoadPosition();
//    RoadPosition roadPosition = RoadUtils::sameRoadPosition();

    _car->setRoadPosition(roadPosition);
//    Vector3d pos = _car->getPosition();

    //pos.z = 2;
    //_car->setPosition(pos);
//    _mobilityModel->SetPosition(toNS3Vector(pos));
    // ToDo: update initial positon in mobility model

    _car->initialize();
    Vector3d pos = _car->getPosition();
    _mobilityModel->SetPosition(toNS3Vector(pos));

    if (uavNextToCar) {
        std::string id = _car->getId();
        id.erase(0, 1);
        std::string uavId = "U"+id;
        std::cout << "Placing near UAV : "<< "U"+id << std::endl;
        Vehicle * uav = VehicleManager::getInstance()->getVehicle(uavId);
        Vector3d pos = uav->getPosition();
        pos.x = _car->getPosition().x;
        pos.y = _car->getPosition().y;
        uav->setPosition(pos);
    }
    return _car;
}

Vehicle *UAVSetup(UAV *_uav, Ptr<ns3::MobilityModel> _mobilityModel)
{
    CallbackValue callbackValue;
    Callback<Behavior*> createBehaviorCallback;
    _mobilityModel->GetAttribute("BehaviorFactory", callbackValue);
    callbackValue.GetAccessor(createBehaviorCallback);

    if (!createBehaviorCallback.IsNull()) {
        Behavior* behavior = createBehaviorCallback();
        _uav->setBehavior(behavior);
    }
    return _uav;
}

void setUAVNextToCar(bool _on)
{
    uavNextToCar = _on;
}

void wireUpLIMoSimAndStartSimulation()
{
//    static bool simulationRan = false;
    if (!LIMoSim::Simulation::getInstance()->running() /*&& !simulationRan*/) {
        createLIMoSimVehicles();
        registerStaticNodes();
        ui::UiManagerService::getInstance()->setUiDataInitialized(true);
        LIMoSim::Simulation::getInstance()->run();
        cleanup();
//        simulationRan = true;
    } else {
        std::cout <<"Simulation is already running" << std::endl;
    }
}

void startSimulation()
{
    if (!LIMoSim::Simulation::getInstance()->running() /*&& !simulationRan*/) {
        ui::UiManagerService::getInstance()->setUiDataInitialized(true);
        LIMoSim::Simulation::getInstance()->run();
        cleanup();
//        simulationRan = true;
    } else {
        std::cout <<"Simulation is already running" << std::endl;
    }
}

// Animation setup


std::string extractNodeId (std::string _context) {
    unsigned int end = _context.find_first_of("/", 10);
    std::string nodeId = _context.substr(10, end-10);
    return nodeId;
}

namespace TraceCallbacks {
void callbackIpv4Rx (std::string context, Ptr<const Packet> packet, Ptr<Ipv4>, unsigned int ) {
    // delay retrieval
    TimestampTag timestamp;
    // Should never not be found since the sender is adding it, but
    // you never know.
    if (packet->FindFirstMatchingByteTag (timestamp)) {
        Time tx = timestamp.GetTimestamp ();
        Time delay = Simulator::Now () - tx;
        std::string nodeId = extractNodeId(context);
        if (enableCallbackLog) std::cout << "Receiver " << nodeId << " - (" << Mobility::External::VehicleNodeRegistry::getInstance()->getVehicleId(nodeId) <<")" << ": received packet with delay: " << delay << std::endl;
    } else {
        // Reception animation
        if (enableCallbackLog) std::cout << "Rx callback with context:" << context << ", extracted node: Id " << extractNodeId(context) << std::endl;
    }
    UI::AnimationUtils::animateReception(extractNodeId(context));
}

void callbackIpv4Tx (std::string context, Ptr<const Packet> packet, Ptr<Ipv4>, unsigned int ) {
    // Transmission animation
    if (enableCallbackLog) std::cout << "Tx callback with context:" << context << ", extracted node: Id " << extractNodeId(context) << std::endl;
    UI::AnimationUtils::animateTransmission(extractNodeId(context));
}

void callbackIpv4SendOutgoing (std::string context, const Ipv4Header& , Ptr<const Packet> packet, unsigned int ) {
    // Timestamp tagging for delay retrieval at reception
    TimestampTag timestamp;
    timestamp.SetTimestamp (Simulator::Now ());
    packet->AddByteTag (timestamp);
    //if (enableCallbackLog) std::cout << "SendOutgoing callback extracted node: Id " << extractNodeId(context) << std::endl;
}

void callBackLtePdcpEnbRxPdu(std::string context, uint16_t, uint8_t, uint32_t, uint64_t)
{
    std::cout <<"ENB PDU RX with context:" << context << std::endl;
}

void callBackLtePdcpEnbTxPdu(std::string context, uint16_t, uint8_t, uint32_t)
{
    std::cout <<"ENB PDU TX with context:" << context << std::endl;
}

void callBackLtePdcpUeRxPdu(std::string context, uint16_t, uint8_t, uint32_t, uint64_t)
{
    std::cout <<"UE PDU RX with context:" << context << std::endl;
}

void callBackLtePdcpUeTxPdu(std::string context, uint16_t, uint8_t, uint32_t)
{
    std::cout <<"UE PDU TX with context:" << context << std::endl;
}

void callbackLteReportUeMeasurement(std::string path, uint16_t rnti, uint16_t cellId, double rsrp, double rsrq, bool servingCell, uint8_t componentCarrierId)
{
    std::cout << "Ue " << extractNodeId(path) <<": RSRP " << rsrp << std::endl;
}

void callbackLteRecvMeasurement(std::string path, uint64_t imsi, uint16_t cellId, uint16_t rnti, LteRrcSap::MeasurementReport meas)
{
    std::cout << "Ue " << extractNodeId(path) <<": RNTI " << rnti << " RSRP:" << meas.measResults.rsrpResult << std::endl;
}

void callbackLteReportCellRsrpSnriMeasurement(std::string path, uint16_t, uint16_t, double rsrp, double, uint8_t)
{
    std::cout << "Ue " << extractNodeId(path) <<": RSRP " << rsrp << std::endl;
}

}

void setupIpv4TrafficAnimation()
{
    using namespace TraceCallbacks;
    // IPv4
    Config::Connect("/NodeList/*/$ns3::Ipv4L3Protocol/Rx", MakeCallback(&callbackIpv4Rx));
    Config::Connect("/NodeList/*/$ns3::Ipv4L3Protocol/Tx", MakeCallback(&callbackIpv4Tx));
    Config::Connect("/NodeList/*/$ns3::Ipv4L3Protocol/SendOutgoing", MakeCallback(&callbackIpv4SendOutgoing));
}

void setupLtePdcpTrafficAnimation()
{
    using namespace TraceCallbacks;
    // ENBs
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::LteEnbNetDevice/LteEnbRrc/UeMap/*/RadioBearerMap/*/LtePdcp/RxPDU", MakeCallback(&callBackLtePdcpEnbRxPdu));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::LteEnbNetDevice/LteEnbRrc/UeMap/*/RadioBearerMap/*/LtePdcp/TxPDU", MakeCallback(&callBackLtePdcpEnbTxPdu));

    // UEs
//    Config::Connect("/NodeList/*/DeviceList/*/$ns3::LteNetDevice/$ns3::LteUeNetDevice/LteUeRrc/RadioBearerMap/*/LtePdcp/RxPDU", MakeCallback(&callBackLtePdcpUeRxPdu));
    //    Config::Connect("/NodeList/*/DeviceList/*/$ns3::LteUeNetDevice/LteUeRrc/RadioBearerMap/*/LtePdcp/TxPDU", MakeCallback(&callBackLtePdcpUeTxPdu));
}

void setupLteUeMeasurementsCallbacks()
{
    using namespace TraceCallbacks;
    Config::Connect ("/NodeList/*/DeviceList/*/LteUePhy/ReportUeMeasurements",
      MakeCallback (&callbackLteReportUeMeasurement));
    Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/RecvMeasurementReport",
      MakeCallback (&callbackLteRecvMeasurement));
    Config::Connect ("/NodeList/*/DeviceList/*/LteUePhy/ReportCurrentCellRsrpSinr",
      MakeCallback (&callbackLteReportCellRsrpSnriMeasurement));

}

void cleanup()
{
    // The ns3 simualtion is already destroyed in the ns3EventScheduler
    // immediately after the run.
    // So only a LIMoSim cleanup remains
    VehicleManager::getInstance()->clearVehicles();
    Mobility::External::VehicleNodeRegistry::getInstance()->clearRegistry();
}

void setDeterministicCarRouteEnabled(bool _enabled)
{
    VehicleManager::getInstance()->setDeterministicCarPathEnabled(_enabled);
}

}

}
