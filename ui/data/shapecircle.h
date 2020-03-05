#ifndef SHAPECIRCLE_H
#define SHAPECIRCLE_H

#include <memory>

#include "shape.h"

namespace LIMoSim {
namespace UI {
namespace Data {

class ShapeCircle : public Shape
{
public:
    ShapeCircle(Vector3d _position = Vector3d(),
                Orientation3d _orientation = Orientation3d(0));
    ShapeCircle(double _radius_mm,
                std::string _color,
                Vector3d _position = Vector3d(),
                Orientation3d _orientation = Orientation3d(0));

    double getRadius();

private:
    double m_radius_mm;

    // Shape interface
public:
    virtual ShapeType getType() const override;
};

typedef std::shared_ptr<ShapeCircle> ShapeCircle_Ptr;
} // namespace Data
} // namespace UI
} // namespace LIMoSim

#endif // SHAPECIRCLE_H
