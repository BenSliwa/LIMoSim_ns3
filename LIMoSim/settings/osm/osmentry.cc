#include "osmentry.h"


namespace LIMoSim
{

OSMEntry::OSMEntry()
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void OSMEntry::addTag(const std::string &_key, const Variant &_value, DOMElement *_parent)
{
    DOMElement *tag = new DOMElement("tag");
    tag->setAttribute("k", _key);
    tag->setAttribute("v", _value);
    _parent->appendChild(tag);
}

}
