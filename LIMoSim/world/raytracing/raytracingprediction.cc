#include "raytracingprediction.h"

#include "algorithm"
#include "math.h"

#include "raytracing.h"

#include "LIMoSim/world/vehiclemanager.h"
#include <QString>
#include <QTextStream>
#include "ui/export/raytracingheatmap.h"

namespace LIMoSim {
namespace raytracing {

RaytracingPrediction *RaytracingPrediction::getInstance(std::vector<int> _heights, std::vector<double> _predictionHorizons)
{
    static RaytracingPrediction instance(_predictionHorizons, _heights);
    return &instance;
}

bool RaytracingPrediction::isRegistered(std::string _vehicleId)
{
    return m_vehiclesList.count(_vehicleId);
}

void RaytracingPrediction::startMonitoring(std::string _vehicleId)
{
    if(isRegistered(_vehicleId)) {
        return;
    }

    m_vehiclesList.insert(_vehicleId);
    m_vehicleFiles[_vehicleId] = VehicleHeightFileStreamMap::mapped_type();
    m_vehiclePreviousPositions[_vehicleId]= PreviousPositions();

    for(int height : m_heights) {
        std::string filename = "../results/RaytracingPrediction_" +
                Simulation::getInstance()->getName() +
                "_h" + std::to_string(height) + "_Vh" + _vehicleId + "_" +
                std::to_string(Simulation::getInstance()->getRunCount()) + ".txt";
        m_vehicleFiles.at(_vehicleId)[height] = new std::ofstream();
        m_vehicleFiles.at(_vehicleId).at(height)->open(filename, std::ios::trunc);
        if (m_vehicleFiles.at(_vehicleId).at(height)->is_open()) {
    //        (*m_vehicleFiles.at(_vehicleId)) << "time,cur_x,cur_y,cur_z,cur_rsrp,rsrp_cm,";
    //        for (double future: m_predictionHorizons) {
    //            (*m_vehicleFiles.at(_vehicleId)) << "pre_x,pre_y,pre_z" << ",";
    //            (*m_vehicleFiles.at(_vehicleId)) << "pre_rsrp" << std::to_string((int)future) << ",";
    //        }
        } else {
            std::cout << "error in raytracing prediction export - could not open export file: " << filename << std::endl;
        }
    }

}

void RaytracingPrediction::stopMonitoring(std::string _vehicleId)
{
    if(isRegistered(_vehicleId)) {
        m_vehiclesList.erase(_vehicleId);
    }
    if (m_vehicleFiles.count(_vehicleId)) {
        for (int height: m_heights){
            m_vehicleFiles.at(_vehicleId).at(height)->close();
            delete m_vehicleFiles.at(_vehicleId).at(height);
        }
        m_vehicleFiles.erase(_vehicleId);
    }
}

void RaytracingPrediction::reset()
{
    if (m_timer) {
        deleteEvent(m_timer);
        m_timer = nullptr;
        initialize();
    }

    for (VehicleHeightFileStreamMap::value_type pair : m_vehicleFiles) {
        for (int height: m_heights){
            pair.second.at(height)->close();
            delete pair.second.at(height);
        }
    }

    m_vehicleFiles.clear();
    m_vehiclesList.clear();
}

void RaytracingPrediction::predictAndMeasure(double _timestamp)
{
    RayTracingHeatMap hm;
    Antenna antenna(m_tx, 0);


    for(int height : m_heights){
        for (std::string vehicleId : m_vehiclesList) {

            Vector3d currentPosition = VehicleManager::getInstance()->getVehicle(vehicleId)->getPosition();
            currentPosition.z = height;
            ConnectivityMap *cm = m_connectivityMaps[height];
            double rsrp = hm.computeRSRP(antenna, currentPosition);
            double rsrp_cm = cm->getEntry(currentPosition);


            QString line;
            QTextStream stream(&line);
            stream << _timestamp << ",";
            stream << QString::fromStdString(currentPosition.toString()) << ",";
            stream << rsrp << "," << rsrp_cm;


            for (double future: m_predictionHorizons) {
                Vector3d predictedPosition = predictPosition(vehicleId, future);
                predictedPosition.z = height;
                double predictedRSRP = cm->getEntry(predictedPosition);

                stream << "," << QString::fromStdString(predictedPosition.toString()) << ",";
                stream << predictedRSRP;
            }

            (*m_vehicleFiles.at(vehicleId).at(height)) << line.toStdString() << std::endl;

        }
    }
}

Vector3d RaytracingPrediction::predictPosition(std::string _vehicleId, double _future)
{

    Vehicle * vehicle = VehicleManager::getInstance()->getVehicle(_vehicleId);

    if (vehicle->getType() == "UAV" &&
            dynamic_cast<UAV*>(vehicle)->getModel()->getBehaviorName()=="RandomWaypoint") {

        std::vector<Vector3d> previousPositions;
        previousPositions.push_back(vehicle->getPosition() + vehicle->getAcceleration() * 1);
        Vector3d waypoint = dynamic_cast<UAV*>(vehicle)->getModel()->getWaypoint();

        for (int step = 1; step < _future; step++) {

            Vector3d prevPos = previousPositions.back();

            if ((prevPos - waypoint).norm() > 30) {
                // waypoint prediction
                Vector3d stepPos = prevPos +
                        ((waypoint - prevPos).norm() ? ((waypoint - prevPos)/(waypoint - prevPos).norm())  : Vector3d())
                        * vehicle->getVelocity().norm() * 1;
                previousPositions.push_back(stepPos);
            } else  {
                // extrapolation
                Vector3d v;
                std::size_t numPrevSteps = previousPositions.size();
                for (std::size_t i = 1; i < numPrevSteps; i++) {
                    v = v + ((previousPositions.at(i) - previousPositions.at(i-1)).norm() ?
                                (previousPositions.at(i) - previousPositions.at(i-1))/(previousPositions.at(i) - previousPositions.at(i-1)).norm():
                                Vector3d());
                }
                v = v/numPrevSteps + prevPos;
                previousPositions.push_back(v);
            }
        }

        return previousPositions.back();


//            Vector3d currPos = m_vehiclePreviousPositions.at(_vehicleId).front();
//            return currPos + ((waypoint - currPos)/(waypoint - currPos).norm()) * vehicle->getVelocity().norm() * _future;
    } else {
        return Vector3d();



        // simple position prediction
//        PreviousPositions prevPos = m_vehiclePreviousPositions.at(_vehicleId);
//        std::size_t numPos = prevPos.size();
//        PreviousPositions::iterator it = prevPos.begin();
//        Vector3d v;
//        for (unsigned int i=1; i < numPos; i++) {
//            std::advance(it,i-1);
//            Vector3d pos_a = *(it++);
//            Vector3d pos_b = *(it);
//            v = v + (pos_a - pos_b)/(pos_a - pos_b).norm();
//        }
//        v = v/numPos;

//        return prevPos.front() + v;
    }
}

void RaytracingPrediction::initialize()
{
    m_timer = new Event(1, this, "second");
    scheduleEvent(m_timer, 0);
}

void RaytracingPrediction::handleEvent(Event *_event)
{
    if(_event->getInfo() == m_timer->getInfo()) {
//        std::cout <<"t=" << _event->getTimestamp() <<std::endl;
        predictAndMeasure(_event->getTimestamp());
        scheduleEvent(m_timer, 1);
    }
}

RaytracingPrediction::RaytracingPrediction(std::vector<double> _predictionHorizons, std::vector<int> _heights):
    m_predictionHorizons(_predictionHorizons),
    m_heights(_heights)
{
    initialize();
    World *world = World::getInstance();
    Vector3d min = Vector3d(world->getBoxMin().x, world->getBoxMin().y, 20);
    Vector3d max = Vector3d(world->getBoxMax().x, world->getBoxMax().y, 20);

    m_tx = (min + max) / 2;
    m_tx.z = 25;

    for(int height : m_heights)
    {
        QString file = "heatmap_" + QString::number(height) + ".txt";
        ConnectivityMap *cm = new ConnectivityMap();
        cm->load(file.toStdString());

        m_connectivityMaps[height] = cm;
    }
}

RaytracingPrediction::~RaytracingPrediction()
{
    if (m_timer) {
        deleteEvent(m_timer);
        m_timer = nullptr;
    }

    for (VehicleHeightFileStreamMap::value_type pair : m_vehicleFiles) {
        for (int height: m_heights){
            pair.second.at(height)->close();
            delete pair.second.at(height);
        }
    }
    m_vehicleFiles.clear();
}

} // namespace raytracing
} // namespace LIMoSim
