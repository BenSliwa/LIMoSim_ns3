#ifndef SHAPECUBE_H
#define SHAPECUBE_H

#include <memory>
#include <functional>

#include "shape.h"
#include "LIMoSim/world/vector3d.h"

namespace LIMoSim {
namespace UI {
namespace Data {


class ShapeCube : public Shape
{
public:
    ShapeCube(Vector3d _position = Vector3d(),
              Orientation3d _orientation = Orientation3d(0));

    ShapeCube(std::string _color);

    ShapeCube(std::function<double(void)> _back,
              std::function<double(void)> _left,
              double _height = 1.5,
              Vector3d _position = Vector3d(),
              Orientation3d _orientation = Orientation3d(0));

    ShapeCube(std::function<double(void)> _back,
              std::function<double(void)> _left,
              double _height,
              std::string _color,
              Vector3d _position = Vector3d(),
              Orientation3d _orientation = Orientation3d(0));


    double getBack();
    double getLeft();
    double getHeight();

private:

    std::function <double(void)> m_back;
    std::function <double(void)> m_left;
    double m_height;

    // Shape interface
public:
    ShapeType getType() const;
};

typedef std::shared_ptr<ShapeCube> ShapeCube_Ptr;
}
}
} // namespace LIMoSim

#endif // SHAPECUBE_H
