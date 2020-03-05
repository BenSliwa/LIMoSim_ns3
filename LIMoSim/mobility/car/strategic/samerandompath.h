#ifndef SAMERANDOMPATH_H
#define SAMERANDOMPATH_H

#include "followpath.h"

namespace LIMoSim {

class SameRandomPath : public FollowPath
{
public:
    SameRandomPath(Car *_car);
    virtual ~SameRandomPath();

    // StrategicModel interface
public:
    void initialize() override;
    void handleNodeReached(Node *_node) override;
    void handleGateReached(Gate *_gate, Intersection *_intersection, LaneSegment *_lane) override;

private:
    Gate* computeRandomDestination();
    Gate* m_start;
    Gate* m_destination;
    bool m_reverse;
    std::vector<Gate*> m_path;
};

} // namespace LIMoSim

#endif // SAMERANDOMPATH_H
