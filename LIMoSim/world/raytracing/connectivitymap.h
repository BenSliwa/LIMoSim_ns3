#ifndef LIMOSIM_CONNECTIVITYMAP_H
#define LIMOSIM_CONNECTIVITYMAP_H

#include "LIMoSim/settings/filehandler.h"
#include "LIMoSim/world/vector3d.h"
#include <map>

namespace LIMoSim
{

class ConnectivityMap
{
public:
    ConnectivityMap();

    void load(const std::string &_file);
    std::string computeCellId(const Vector3d &_position);
    bool hasEntry(const std::string &_cell);
    double getEntry(const std::string &_cell);
    double getEntry(const Vector3d &_position);

private:
    std::map<std::string, double> m_cells;
    double m_cellWidth_m;
};

}

#endif // LIMOSIM_CONNECTIVITYMAP_H
