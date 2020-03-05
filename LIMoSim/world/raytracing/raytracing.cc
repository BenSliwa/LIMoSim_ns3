#include "raytracing.h"
#include <iostream>
#include <algorithm>

#include "LIMoSim/world/worldutils.h"


namespace LIMoSim
{

Raytracing::Raytracing()
{
}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

RayTrace Raytracing::trace(const Vector3d &_from, const Vector3d &_to)
{


    RayTrace trace;
//    if (isCached(_from, _to, &trace)) {
//        return trace;
//    }
    trace.distance_m = (_to - _from).norm();
    trace.attenuated_m = 0;


    std::map<std::string, Building*> buildings = World::getInstance()->getBuildings();
    for(std::map<std::string, Building*>::iterator it=buildings.begin(); it!=buildings.end(); it++)
    {
        Building *building = it->second;
        if (!isBuildingOnPath(_from, _to, building)) {
            continue;
        }

        std::vector<RayIntersection> buildingIntersections = computeBuildingIntersections(_from, _to, building);

        // trace the height component (x-layer: tx/rx line, y-layer: z)
        if(buildingIntersections.size()>0)
        {
            int s = buildingIntersections.size();
            bool inside = (s%2==1);
            if(inside)
                s = 1 + s/2;
            else
                s = s/2;

            for(int i=0; i<s; i++)
            {
                bool nowInside = !(!inside || i<s-1);
                RayIntersection p0 = buildingIntersections.at(i*2);
                RayIntersection p1;
                if(!nowInside)
                    p1 = buildingIntersections.at(i*2+1);
                else // the target point is within an obstacle
                {
                    p1.intersect = true;
                    p1.point = _to;
                    p1.distance_m = (Vector3d(_to.x, _to.y, 0) - Vector3d(_from.x, _from.y, 0)).norm();
                }

                std::vector<RayIntersection> result = checkZIntersection(_from, _to, p0.distance_m, p1.distance_m, building->getHeight());
                if(nowInside && result.size()>0)
                {
                    insertIntersection(trace.intersections, result.at(0));
                    trace.attenuated_m += (result.at(0).point - _to).norm();
                }
                else if(!nowInside && result.size()==2)
                {
                    insertIntersection(trace.intersections, result.at(0));
                    insertIntersection(trace.intersections, result.at(1));

                    trace.attenuated_m += (result.at(0).point - result.at(1).point).norm();
                }
            }
        }
    }
    //    std::cout << "total: " << trace.distance_m << "\t" << "attenuated: " << trace.attenuated_m << "\t" << "entries: " << trace.intersections.size() << std::endl;

//    cache(_from, _to, trace);
    return trace;
}

std::vector<RayIntersection> Raytracing::checkZIntersection(const Vector3d &_from, const Vector3d &_to, double _d0, double _d1, double _h)
{
    std::vector<RayIntersection> intersections;

    Vector3d from(0, _from.z, 0);
    Vector3d to((_to-_from).norm(), _to.z, 0);
    Vector3d dir = (to - from).normed();

    Vector3d p0(_d0, _h, 0);
    Vector3d p1(_d1, _h, 0);

    std::vector<RayIntersection> candidates;
    candidates.push_back(checkIntersection(from, dir, Vector3d(p0.x), p0));
    candidates.push_back(checkIntersection(from, dir, p0, p1));
    candidates.push_back(checkIntersection(from, dir, Vector3d(p1.x), p1));

    for(unsigned int i=0; i<candidates.size(); i++)
    {
        RayIntersection candidate = candidates.at(i);
        if(candidate.intersect)
        {
            // tranform back to xyz
            Vector3d dir = (_to - _from).normed();
            double distance2d_m = candidate.point.x;
            candidate.point = _from + dir * distance2d_m;
            candidate.distance_m = distance2d_m;
            intersections.push_back(candidate);
        }
    }

    return intersections;
}

void Raytracing::clearCache()
{
    m_cache.clear();
}

/*************************************
 *           PRIVATE METHODS         *
 ************************************/

std::vector<RayIntersection> Raytracing::computeBuildingIntersections(const Vector3d &_from, const Vector3d &_to, Building *_building)
{
    std::vector<RayIntersection> intersections;
    std::vector<Node*> nodes = _building->getNodes();
    for(int i=0; i<nodes.size(); i++) // for all segments
    {
        Vector3d v0 = nodes.at(i)->getPosition();
        Vector3d v1 = nodes.at((i+1)%nodes.size())->getPosition();

        bool isOnSegment = false;
        if((v0-_from).norm()+(v1-_from).norm()==(v1-v0).norm())
            isOnSegment = true;

        double distance2d_m = (Vector3d(_to.x, _to.y)-Vector3d(_from.x, _from.y)).norm();
        RayIntersection entry = checkIntersection(_from, _to-_from, v0, v1);
        if(entry.intersect && entry.distance_m<distance2d_m && !isOnSegment)
        {
            insertIntersection(intersections, entry);
        }
    }

    return intersections;
}

RayIntersection Raytracing::checkIntersection(const Vector3d &_origin, const Vector3d &_dir, const Vector3d &_p0, const Vector3d &_p1)
{
    Vector3d v1 = _origin - _p0; v1.z = 0;
    Vector3d v2 = _p1 - _p0; v2.z = 0;
    Vector3d v3(-_dir.y, _dir.x);

    double d = cross(v2, v1);
    double t1 = d / (v2 * v3);
    double t2 = (v1 * v3) / (v2 * v3);

    RayIntersection entry;
    entry.intersect = false;
    if(t1>=0 && t2<=1 && t2>=0)
    {
        entry.intersect = true;
        entry.point = _origin + _dir * t1;
        entry.distance_m = (entry.point - _origin).norm();
    }

    return entry;
}

void Raytracing::insertIntersection(std::vector<RayIntersection> &_intersections, const RayIntersection &_entry)
{
    if(_intersections.size()==0)
        _intersections.push_back(_entry);
    else
    {
        if(_intersections.at(_intersections.size()-1).distance_m<=_entry.distance_m)
            _intersections.push_back(_entry);
        else
        {
            for(int i=0; i<_intersections.size(); i++)
            {
                double distance_m = _intersections.at(i).distance_m;
                if(_entry.distance_m<distance_m)
                {
                    _intersections.insert(_intersections.begin() + i, _entry);
                    break;
                }
            }
        }
    }
}

double Raytracing::cross(const Vector3d &_v0, const Vector3d &_v1)
{
    return (_v0.x*_v1.y) - (_v0.y*_v1.x);
}

bool Raytracing::isBuildingOnPath(const Vector3d &_from, const Vector3d &_to, Building *_building)
{
    std::vector<Node*> nodes = _building->getNodes();

    for (uint nodeIndex = 0; nodeIndex < nodes.size()-1; nodeIndex++) {
        if (world::utils::isIntersecting(_from, _to, nodes.at(nodeIndex)->getPosition(), nodes.at(nodeIndex+1)->getPosition())) {
            return true;
        }
    }
    return false;
}

bool Raytracing::isCached(const Vector3d &_from, const Vector3d &_to, RayTrace* _value)
{
    //    m_cache.count(TraceKey(_from, _to));
    auto it = std::find_if(m_cache.begin(),
                     m_cache.end(),
                     [_from, _to] (std::pair<TraceKey, std::pair<RayTrace, uint>> c) {
            return c.first.from == _from && c.first.to == _to;
        });

    if (it != m_cache.end()) {
        (*_value = (it->second.first));
        return true;
    }
    return false;
}

RayTrace Raytracing::getFromCache(const Vector3d &_from, const Vector3d &_to)
{
    m_cache.at(TraceKey(_from, _to)).second = m_cache.at(TraceKey(_from, _to)).second + 1;
    return m_cache.at(TraceKey(_from, _to)).first;

}

void Raytracing::cache(const Vector3d &_from, const Vector3d &_to, RayTrace _trace)
{
    m_cache[TraceKey(_from, _to)] = std::make_pair(_trace,0);
}

bool operator < (const TraceKey &_lhs, const TraceKey &_rhs) {
    return (_lhs.from.toString() + _lhs.to.toString()) < (_rhs.from.toString() + _rhs.to.toString());
}

bool operator == (const TraceKey &_lhs, const TraceKey &_rhs) {

    return (_lhs.from - _rhs.from).norm() < 20 && (_lhs.to - _rhs.to).norm() < 20;
//    return (_lhs.from == _rhs.from) && (_lhs.to == _rhs.to);

//    double tolerance = 10.0;
//    bool sameFrom = false;
//    bool sameTo = false;

//    Vector3d diff = (_lhs.from - _rhs.from).cAbs();
//    sameFrom = diff.x < tolerance && diff.y < tolerance && diff.z < tolerance;

//    diff = (_lhs.to - _rhs.to).cAbs();
//    sameTo = diff.x < tolerance && diff.y < tolerance && diff.z < tolerance;
//    return sameFrom && sameTo;
}


}
