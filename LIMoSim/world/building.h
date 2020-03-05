#ifndef LIMOSIM_BUILDING_H
#define LIMOSIM_BUILDING_H

#include "way.h"

namespace LIMoSim
{

class Building : public Way
{
public:
    Building(const std::string &_id);

    //
    void setName(const std::string &_name);
    void setHeight(double _height_m);
    std::string getName();
    double getHeight();

private:
    std::string m_name;
    double m_height_m;
};

}

#endif // BUILDING_H
