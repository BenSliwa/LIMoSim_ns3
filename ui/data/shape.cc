#include "shape.h"

namespace LIMoSim {
namespace UI {
namespace Data {

Shape::Shape(Vector3d _position, Orientation3d _orientation, std::string _color):
    m_color(_color),
    m_position(_position),
    m_orientation(_orientation)
{

}

Shape::~Shape()
{
}

std::string Shape::getColor()
{
    return m_color;
}

Orientation3d Shape::getOrientation()
{
    return m_orientation;
}

Vector3d Shape::getPosition()
{
    return m_position;
}

void Shape::setColor(std::string _color)
{
    m_color = _color;
}

void Shape::setOrientation(Orientation3d _orientation)
{
    m_orientation = _orientation;
}

void Shape::setPosition(Vector3d _pos)
{
    m_position = _pos;
}


}
}
}
