#ifndef VECTOR_H
#define VECTOR_H

#include <vector>
#include <algorithm>
#include <functional>

namespace LIMoSim {
namespace utils {
namespace vector {

template <class U>
bool contains(std::vector<U> _vec, U _val) {
    return std::find(_vec.begin(), _vec.end(), _val) != _vec.end();
}

template <class U>
int indexOf(std::vector<U> _vec, U _val) {
    auto it = std::find(_vec.begin(), _vec.end(), _val);
    if (it != _vec.end()) {
        int index = std::distance(_vec.begin(), it);
        return index;
    } else {
        return -1;
    }
}

template <class U>
int indexOf(std::vector<U> _vec, U _val, int _startOffset) {
    typename std::vector<U>::iterator start = _vec.begin();
    std::advance(start, _startOffset);
    auto it = std::find(start, _vec.end(), _val);
    if (it != _vec.end()) {
        int index =  std::distance(_vec.begin(), it);
        return index;
    } else {
        return -1;
    }
}

template <class U, class _Predicate>
int indexOf(std::vector<U> _vec, _Predicate _predicate) {
    auto it = std::find_if(_vec.begin(),_vec.end(),_predicate);
    if (it != _vec.end()) {
        int index = std::distance(_vec.begin(), it);
        return index;
    } else {
        return -1;
    }
}

} // namespace vector

} // namespace utils
} // namespace LIMoSim


#endif // VECTOR_H
