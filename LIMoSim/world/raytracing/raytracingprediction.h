#ifndef RAYTRACINGPREDICTION_H
#define RAYTRACINGPREDICTION_H

#include <map>
#include <set>
#include <list>

#include "LIMoSim/world/world.h"
#include "LIMoSim/world/raytracing/connectivitymap.h"
#include "LIMoSim/simulation/eventhandler.h"
#include <QMap>

namespace LIMoSim {
namespace raytracing {

struct RSRP_Prediction {
    double rsrp;
    Vector3d position;
};

typedef std::map<double, RSRP_Prediction> RSRP_Predictions;
typedef std::list<Vector3d> PreviousPositions;
typedef std::map<int, std::ofstream*> HeightStreamMap;
typedef std::map<std::string, HeightStreamMap> VehicleHeightFileStreamMap;

class RaytracingPrediction: public EventHandler
{
public:
    static RaytracingPrediction* getInstance(std::vector<int> _heights = {5}, std::vector<double> _predictionHorizons = {5});

    // Monitoring control
    bool isRegistered(std::string _vehicleId);
    void startMonitoring(std::string _vehicleId);
    void stopMonitoring(std::string _vehicleId);
    void reset();

    // Raytracing prediction
    void predictAndMeasure(double _timestamp);
    Vector3d predictPosition(std::string _vehicleId, double _future);

    // EventHandler interface
public:
    virtual void initialize() override;
    virtual void handleEvent(Event *_event) override;

private:
    std::set<std::string> m_vehiclesList;
    std::map<std::string, PreviousPositions> m_vehiclePreviousPositions;
    VehicleHeightFileStreamMap m_vehicleFiles;
    QMap<int, ConnectivityMap*> m_connectivityMaps;
    Vector3d m_tx;
    unsigned int m_positionMemory = 10;
    double m_future = 60.0;
    double m_h;
    std::vector<double> m_predictionHorizons {1};
    std::vector<int> m_heights;
    Event *m_timer;

    RaytracingPrediction(std::vector<double> _predictionHorizons, std::vector<int> _heights);
    ~RaytracingPrediction();
};

} // namespace raytracing
} // namespace LIMoSim

#endif // RAYTRACINGPREDICTION_H
