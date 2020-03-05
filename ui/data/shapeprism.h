#ifndef SHAPEPRISM_H
#define SHAPEPRISM_H

#include <memory>

#include "shape.h"

namespace LIMoSim {
namespace UI {
namespace Data {

class ShapePrism : public Shape
{
public:
    ShapePrism(Vector3d _position = Vector3d(),
               Orientation3d _orientation = Orientation3d(0));
    ShapePrism(std::string _color);
    ShapePrism(double _base_mm, double _depth_mm, double _height_mm,
               Vector3d _position = Vector3d(),
               Orientation3d _orientation = Orientation3d(0));
    ShapePrism(double _base_mm, double _depth_mm, double _height_mm,
               std::string _color,
               Vector3d _position = Vector3d(),
               Orientation3d _orientation = Orientation3d(0));

    double getBase();
    double getDepth();
    double getHeight();

    void setBase(double _base_mm);
    void setDepth(double _depth_mm);
    void setHeight(double _height_mm);


private:
    double m_base_mm;
    double m_depth_mm;
    double m_height_mm;

    // Shape interface
public:
    virtual ShapeType getType() const override;
};

typedef std::shared_ptr<ShapePrism> ShapePrism_Ptr;
}
}
} // namespace LIMoSim

#endif // SHAPEPRISM_H
