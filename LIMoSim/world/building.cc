#include "building.h"

namespace LIMoSim
{

Building::Building(const std::string &_id) :
    Way(_id, "Building"),
    m_height_m(10)
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void Building::setName(const std::string &_name)
{
    m_name = _name;
}

void Building::setHeight(double _height_m)
{
    m_height_m = _height_m;
}

std::string Building::getName()
{
    return m_name;
}

double Building::getHeight()
{
    return m_height_m;
}

}
