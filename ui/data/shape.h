#ifndef SHAPE_H
#define SHAPE_H

#include <string>
#include <memory>
#include <vector>

#include "LIMoSim/world/vector3d.h"
#include "LIMoSim/world/orientation3d.h"

namespace LIMoSim {
namespace UI {
namespace Data {
enum ShapeType { Cube, Prism, Circle };

class Shape {
public:
    Shape( Vector3d _position = Vector3d(),
           Orientation3d _orientation = Orientation3d(0),
           std::string _color = "white");
    virtual ~Shape();

    virtual ShapeType getType() const = 0;

    std::string getColor();
    Orientation3d getOrientation();
    Vector3d getPosition();


    void setColor(std::string _color);
    void setOrientation(Orientation3d _orientation);
    void setPosition(Vector3d _pos);

private:
    std::string m_color;
    /**
     * @brief m_position
     * Shape position relative to object's main position
     */
    Vector3d m_position;
    /**
     * @brief m_orientation
     * Shape orientation relative to object's main orientation
     */
    Orientation3d m_orientation;
};

typedef std::shared_ptr<Shape> Shape_Ptr;
typedef std::vector<Shape_Ptr> Shape_Ptrs;

}
}
}
#endif // SHAPE_H
