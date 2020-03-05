#ifndef LIMOSIM_AREA_H
#define LIMOSIM_AREA_H

#include "way.h"

namespace LIMoSim
{

class Area : public Way
{
public:
    Area(const std::string &_id);

    void setAreaType(const std::string &_type);

    std::string getAreaType();

private:
    std::string m_areaType;
};

}


#endif // LIMOSIM_AREA_H
