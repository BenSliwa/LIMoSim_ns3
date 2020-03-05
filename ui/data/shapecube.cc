#include "shapecube.h"

namespace LIMoSim {
namespace UI {
namespace Data {

ShapeCube::ShapeCube(Vector3d _position, Orientation3d _orientation):
    Shape(_position, _orientation),
    m_back([]() {
        return 2.5;
    }),
    m_left([]() {
        return 5.0;
    }),
    m_height(1.5)
{

}

ShapeCube::ShapeCube(std::string _color):
    Shape(Vector3d(), Orientation3d(0), _color),
    m_back([]() {
        return 2.5;
    }),
    m_left([]() {
        return 5.0;
    }),
    m_height(1.5)
{

}

ShapeCube::ShapeCube(std::function<double(void)> _back,
                     std::function<double(void)> _left,
                     double _height,
                     Vector3d _position,
                     Orientation3d _orientation):
    Shape(_position, _orientation),
    m_back(_back),
    m_left(_left),
    m_height(_height)
{

}

ShapeCube::ShapeCube(std::function<double(void)> _back,
                     std::function<double(void)> _left,
                     double _height,
                     std::string _color,
                     Vector3d _position,
                     Orientation3d _orientation):
    Shape(_position, _orientation, _color),
    m_back(_back),
    m_left(_left),
    m_height(_height)
{

}

double ShapeCube::getBack()
{
    return m_back();
}

double ShapeCube::getLeft()
{
    return m_left();
}

double ShapeCube::getHeight()
{
    return m_height;
}

ShapeType ShapeCube::getType() const
{
    return ShapeType::Cube;
}

}
}
} // namespace LIMoSim

