#include "area.h"

namespace LIMoSim
{

Area::Area(const std::string &_id) :
    Way(_id, "Area")
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void Area::setAreaType(const std::string &_type)
{
    m_areaType = _type;
}

std::string Area::getAreaType()
{
    return m_areaType;
}

}
