#include "shapeprism.h"

namespace LIMoSim {
namespace UI {
namespace Data {

ShapePrism::ShapePrism(Vector3d _position, Orientation3d _orientation):
    Shape(_position, _orientation),
    m_base_mm(4.0),
    m_depth_mm(0.5),
    m_height_mm(5.0)
{

}

ShapePrism::ShapePrism(std::string _color):
    Shape(Vector3d(), Orientation3d(0), _color),
    m_base_mm(4.0),
    m_depth_mm(0.5),
    m_height_mm(5.0)
{

}

ShapePrism::ShapePrism(double _base_mm,
                       double _depth_mm,
                       double _height_mm,
                       Vector3d _position,
                       Orientation3d _orientation):
    Shape(_position, _orientation),
    m_base_mm(_base_mm),
    m_depth_mm(_depth_mm),
    m_height_mm(_height_mm)
{

}

ShapePrism::ShapePrism(double _base_mm,
                       double _depth_mm,
                       double _height_mm,
                       std::string _color,
                       Vector3d _position,
                       Orientation3d _orientation):
    Shape(_position, _orientation, _color),
    m_base_mm(_base_mm),
    m_depth_mm(_depth_mm),
    m_height_mm(_height_mm)
{

}

double ShapePrism::getBase()
{
    return m_base_mm;
}

double ShapePrism::getDepth()
{
    return m_depth_mm;
}

double ShapePrism::getHeight()
{
    return m_height_mm;
}

void ShapePrism::setBase(double _base_mm)
{
    m_base_mm = _base_mm;
}

void ShapePrism::setDepth(double _depth_mm)
{
    m_depth_mm = _depth_mm;
}

void ShapePrism::setHeight(double _height_mm)
{
    m_height_mm = _height_mm;
}




ShapeType ShapePrism::getType() const
{
    return ShapeType::Prism;
}

}
}
} // namespace LIMoSim
