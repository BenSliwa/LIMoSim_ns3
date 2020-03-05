#ifndef DETERMINISTICPATH_H
#define DETERMINISTICPATH_H


#include "followpath.h"

namespace LIMoSim {

struct GateInfo {
    std::string nodeId;
    std::string roadId;
    int segmentIndex;
    GateInfo(std::string _nodeId, std::string _roadId, int _segmentIndex):
        nodeId(_nodeId),
        roadId(_roadId),
        segmentIndex(_segmentIndex)
    {}
};

class DeterministicPath : public FollowPath
{
public:
    DeterministicPath(Car *_car);
    virtual ~DeterministicPath();


    // StrategicModel interface
public:
    void initialize();
    void handleNodeReached(Node *_node);
    void handleGateReached(Gate *_gate, Intersection *_intersection, LaneSegment *_lane);

    std::vector<GateInfo> getGatesData();

protected:
    void clearGateData();
    void addGateData(GateInfo _gateInfo);
    void setPath(std::vector<Gate*> _path);

private:

    std::vector<Gate*> m_path;
    std::vector<GateInfo> m_gatesData;
};

} // namespace LIMoSim

#endif // DETERMINISTICPATH_H
