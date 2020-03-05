#ifndef LIMOSIM_OSMENTRY_H
#define LIMOSIM_OSMENTRY_H

#include "LIMoSim/settings/domelement.h"

namespace LIMoSim
{

class OSMEntry
{
public:
    OSMEntry();

    static void addTag(const std::string &_key, const Variant &_value, DOMElement *_parent);
};

}
#endif // LIMOSIM_OSMENTRY_H
