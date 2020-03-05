#include "worldobject.h"

WorldObject::WorldObject(const std::string &_type, const std::string &_id) :
    m_type(_type),
    m_id(_id)
{

}


/*************************************
 *            PUBLIC METHODS         *
 ************************************/

std::string WorldObject::getType()
{
    return m_type;
}

std::string WorldObject::getId()
{
    return m_id;
}

std::string WorldObject::toString()
{
    return m_id;
}

void WorldObject::info()
{
    std::cout << toString() << std::endl;
}
