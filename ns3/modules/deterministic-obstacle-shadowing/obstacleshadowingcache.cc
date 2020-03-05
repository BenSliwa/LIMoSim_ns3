#include "obstacleshadowingcache.h"

#include <algorithm>

namespace LIMoSim {
namespace NS3 {
namespace Modules {
namespace ObstacleShadowing {

ObstacleShadowingCache *ObstacleShadowingCache::getInstance()
{
    static ObstacleShadowingCache instance;
    return &instance;
}

void ObstacleShadowingCache::cache(const std::string &_key, double & _value)
{
    m_cache[_key] = _value;
    if (m_cache.size() > maxCacheSize) {
        m_cache.clear();
    }
}

void ObstacleShadowingCache::clear()
{
    m_cache.clear();
}

bool ObstacleShadowingCache::counts(const std::string & _cacheKeyVal, double* _val)
{
    auto it = m_cache.find(_cacheKeyVal);

    if (it != m_cache.end()) {
        (*_val = (it->second));
        return true;
    }
    return false;
}

double ObstacleShadowingCache::get(const std::string &_key)
{
    return m_cache.at(_key);
}

ObstacleShadowingCache::ObstacleShadowingCache()
{

}

} // namespace ObstacleShadowing
} // namespace Modules
} // namespace NS3
} // namespace LIMoSim
