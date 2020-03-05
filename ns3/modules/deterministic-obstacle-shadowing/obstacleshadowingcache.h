#ifndef OBSTACLESHADOWINGCACHE_H
#define OBSTACLESHADOWINGCACHE_H

#include <map>
#include "LIMoSim/world/vector3d.h"


namespace LIMoSim {
namespace NS3 {
namespace Modules {
namespace ObstacleShadowing {

class ObstacleShadowingCache
{
public:
    static ObstacleShadowingCache* getInstance();

    void cache(const std::string &_key, double &_value);
    void clear();
    bool counts(const std::string & _cacheKeyVal, double *_val);
    double get(const std::string & _key);

private:
    std::map<std::string, double> m_cache;

    unsigned int maxCacheSize = 150;


    ObstacleShadowingCache();
};

} // namespace ObstacleShadowing
} // namespace Modules
} // namespace NS3
} // namespace LIMoSim

#endif // OBSTACLESHADOWINGCACHE_H
