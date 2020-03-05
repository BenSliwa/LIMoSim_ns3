#include "shapecircle.h"

namespace LIMoSim {
namespace UI {
namespace Data {

ShapeCircle::ShapeCircle(Vector3d _position, Orientation3d _orientation):
    Shape(_position, _orientation, "orange"),
    m_radius_mm(5)
{

}

ShapeCircle::ShapeCircle(double _radius_mm,
                         std::string _color,
                         Vector3d _position,
                         Orientation3d _orientation):
    Shape(_position, _orientation, _color),
    m_radius_mm(_radius_mm)
{

}

double ShapeCircle::getRadius()
{
    return m_radius_mm;
}

ShapeType ShapeCircle::getType() const
{
    return ShapeType::Circle;
}

} // namespace Data
} // namespace UI
} // namespace LIMoSim
