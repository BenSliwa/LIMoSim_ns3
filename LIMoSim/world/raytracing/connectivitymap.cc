#include "connectivitymap.h"
#include "LIMoSim/settings/parser.h"
#include "LIMoSim/world/world.h"

namespace LIMoSim
{

ConnectivityMap::ConnectivityMap() :
    m_cellWidth_m(10)
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/


void ConnectivityMap::load(const std::string &_file)
{
    std::vector<std::string> lines = FileHandler::read(_file);
    for(int y=0; y<lines.size(); y++)
    {
        std::vector<std::string> line = Parser::split(lines.at(y), ",");
        for(int x=0; x<line.size(); x++)
        {
            double value = atof(line.at(x).c_str());

            std::stringstream stream;
            stream << x << "," << y;
            m_cells[stream.str()] = value;
        }
    }

    std::cout << "ConnectivityMap::load " << m_cells.size() << std::endl;
}

std::string ConnectivityMap::computeCellId(const Vector3d &_position)
{
    World *world = World::getInstance();
    Vector3d boxMin = world->getBoxMin();

    int x = (_position.x - boxMin.x) / m_cellWidth_m;
    int y = (_position.y - boxMin.y) / m_cellWidth_m;

    std::stringstream stream;
    stream << x << "," << y;

    return stream.str();
}

bool ConnectivityMap::hasEntry(const std::string &_cell)
{
    return m_cells.count(_cell)>0;
}

double ConnectivityMap::getEntry(const std::string &_cell)
{
    return m_cells[_cell];
}

double ConnectivityMap::getEntry(const Vector3d &_position)
{
    std::string cell = computeCellId(_position);
    if(hasEntry(cell))
        return getEntry(cell);
    return 0;
}
/*************************************
 *           PRIVATE METHODS         *
 ************************************/


}
