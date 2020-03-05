#ifndef UIDATAENTRY_H
#define UIDATAENTRY_H

#include <string>
#include <vector>

#include "shape.h"
#include "animations.h"

#include "LIMoSim/world/vector3d.h"
#include "LIMoSim/world/orientation3d.h"

namespace LIMoSim {

namespace UI {

namespace Data {

enum UIDataEntryType  {STATIC, MOBILE};

class UIDataEntry
{
public:
    UIDataEntry(std::string _id, UIDataEntryType _type, Shape_Ptr _shape);
    UIDataEntry(std::string _id, UIDataEntryType _type, Shape_Ptrs _shapes);
    virtual ~UIDataEntry();

    std::string getId();
    UIDataEntryType getType();
    Shape_Ptr getShape();
    Shape_Ptrs getShapes();


    virtual Vector3d getPosition() = 0;
    virtual Orientation3d getOrientation() = 0;


    /**
     * @brief getShapeColor
     * @return ShapeColor mit Rücksicht auf Animationen
     */
    std::string getShapeColor();

    void setShape(Shape_Ptr _shape);
    void setShapes(Shape_Ptrs _shapes);


    void animateReception();
    void animateTransmission();
    void processAnimations();


private:
    std::string m_id;
    UIDataEntryType m_type;

protected:
    Shape_Ptr m_shape;
    Shape_Ptrs m_shapes;
    Animations* m_animations;
};

typedef std::shared_ptr<UIDataEntry> UIDataEntry_ptr;

} // namespace Data

} // namespace UI

}// namespace LIMoSim
#endif // UIDATAENTRY_H
