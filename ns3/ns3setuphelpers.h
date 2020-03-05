#ifndef NS3GLOBALS_H
#define NS3GLOBALS_H

#include <ns3/mobility-helper.h>
#include <ns3/ipv4.h>
#include <ns3/lte-rrc-sap.h>

#include "ns3/ns3utils.h"

namespace LIMoSim {

class Car;
class UAV;
class Vehicle;
struct RoadPosition;

namespace NS3 {

class LimoSimMobilityModel;

void setupNS3Globals();

void setupNS3MobilityFromVehicleProps(
        ns3::MobilityHelper &_mobility,
        std::vector<VehicleProperties> _vehiclesProps,
        NodeContainer &_LIMoSimNodes
        );

void createLIMoSimVehicles();
void registerStaticNodes();

void cleanup();
void setDeterministicCarRouteEnabled(bool _enabled);

Vehicle *carSetup(Car* _car, Ptr<ns3::MobilityModel> _mobilityModel);
Vehicle *UAVSetup(UAV* _uav, Ptr<ns3::MobilityModel> _mobilityModel);

void setUAVNextToCar(bool _on);

void wireUpLIMoSimAndStartSimulation();
void startSimulation();

// Animation setup

extern bool enableAnimationLog;
extern bool enableCallbackLog;

std::string extractNodeId (std::string _context);

namespace TraceCallbacks {
void callbackIpv4Rx (std::string context, Ptr<const Packet> packet, Ptr<Ipv4>, unsigned int );
void callbackIpv4Tx (std::string context, Ptr<const Packet> packet, Ptr<Ipv4>, unsigned int );
void callbackIpv4SendOutgoing (std::string context, const Ipv4Header& , Ptr<const Packet> packet, unsigned int );

void callBackLtePdcpEnbRxPdu (std::string context, uint16_t, uint8_t, uint32_t, uint64_t);
void callBackLtePdcpEnbTxPdu (std::string context, uint16_t, uint8_t, uint32_t);
void callBackLtePdcpUeRxPdu (std::string context, uint16_t, uint8_t, uint32_t, uint64_t);
void callBackLtePdcpUeTxPdu (std::string context, uint16_t, uint8_t, uint32_t);

void callbackLteReportUeMeasurement (std::string path,uint16_t rnti, uint16_t cellId,
                                     double rsrp, double rsrq, bool servingCell, uint8_t componentCarrierId);
void callbackLteRecvMeasurement (std::string path, uint64_t imsi, uint16_t cellId,
                                 uint16_t rnti, LteRrcSap::MeasurementReport meas);
void callbackLteReportCellRsrpSnriMeasurement (std::string path, uint16_t, uint16_t, double, double, uint8_t );
}

void setupIpv4TrafficAnimation();
void setupLtePdcpTrafficAnimation();
void setupLteUeMeasurementsCallbacks();

}

}

#endif // NS3GLOBALS_H
