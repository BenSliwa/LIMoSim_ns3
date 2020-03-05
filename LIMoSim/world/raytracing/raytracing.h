#ifndef LIMOSIM_RAYTRACING_H
#define LIMOSIM_RAYTRACING_H

// TODO: define building attenuation parameters upon creation

#include "LIMoSim/world/world.h"



namespace LIMoSim
{

class Building;

struct TraceKey {
    Vector3d from;
    Vector3d to;
    TraceKey(Vector3d _from, Vector3d _to): from(_from), to(_to){}
};

bool operator < (const TraceKey &_lhs, const TraceKey &_rhs);
bool operator == (const TraceKey &_lhs, const TraceKey &_rhs);


struct RayIntersection
{
    bool intersect;
    Vector3d point;
    double distance_m;
};

struct RayTrace
{
    double distance_m;
    double attenuated_m;
    std::vector<RayIntersection> intersections;
};

class Raytracing
{
public:
    Raytracing();

    RayTrace trace(const Vector3d &_from, const Vector3d &_to);
    std::vector<RayIntersection> checkZIntersection(const Vector3d &_from, const Vector3d &_to, double _d0, double _d1, double _h);
    void clearCache();


private:
    std::vector<RayIntersection> computeBuildingIntersections(const Vector3d &_from, const Vector3d &_to, Building *_building);

    RayIntersection checkIntersection(const Vector3d &_origin, const Vector3d &_dir, const Vector3d &_p0, const Vector3d &_p1);
    void insertIntersection(std::vector<RayIntersection> &_intersections, const RayIntersection &_entry);
    double cross(const Vector3d &_v0, const Vector3d &_v1);

    bool isBuildingOnPath(const Vector3d &_from, const Vector3d &_to, Building* _building);

    bool isCached(const Vector3d &_from, const Vector3d &_to, RayTrace* _value);
    RayTrace getFromCache(const Vector3d &_from, const Vector3d &_to);
    void cache(const Vector3d &_from, const Vector3d &_to, RayTrace _trace);

    std::map<TraceKey, std::pair<RayTrace, uint>> m_cache;
};

}

#endif // LIMOSIM_RAYTRACING_H
