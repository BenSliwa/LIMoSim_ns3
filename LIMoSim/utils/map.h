#ifndef MAP_H
#define MAP_H

#include <map>
#include <vector>

namespace LIMoSim {

namespace utils {

namespace map {

template<class U, class V>
typename std::map< U,V>::iterator findByValue(std::map<U,V> _map, V _value) {
    for (typename std::map<U,V>::iterator entry = _map.begin(); entry != _map.end(); entry++) {
        if ((*entry).second == _value) {
            return entry;
        }
    }
    return _map.end();
};

template<class U, class V>
std::vector<typename std::map< U,V>::iterator> findAllByValue(std::map<U,V> _map, V _value) {
    std::vector<typename std::map<U,V>::iterator> result;
    for (typename std::map<U,V>::iterator entry = _map.begin(); entry != _map.end(); entry++) {
        if ((*entry).second == _value) {
            result.push_back(entry);
        }
    }
    return result;
};



}

}

}

#endif // MAP_H
