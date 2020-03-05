#ifndef LIMOSIM_WORLDOBJECT_H
#define LIMOSIM_WORLDOBJECT_H

#include "vector3d.h"
#include <sstream>
#include <vector>
#include <iostream>

class WorldObject
{
public:
    WorldObject(const std::string &_type, const std::string &_id = "");

    //
    std::string getType();
    std::string getId();

    virtual std::string toString();
    void info();



protected:
    std::string m_type;
    std::string m_id;
};

#endif // LIMOSIM_WORLDOBJECT_H
