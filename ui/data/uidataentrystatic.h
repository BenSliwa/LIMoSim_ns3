#ifndef UIDATAENTRYSTATIC_H
#define UIDATAENTRYSTATIC_H

#include "uidataentry.h"
#include "memory"

#include "LIMoSim/world/vector3d.h"

namespace LIMoSim {

namespace UI {

namespace Data {

class UIDataEntryStatic : public UIDataEntry
{
public:
    UIDataEntryStatic(std::string _id);
    UIDataEntryStatic(std::string _id, Vector3d _position, Orientation3d _orientation);
    UIDataEntryStatic(std::string _id, Vector3d _position, Orientation3d _orientation, Shape_Ptrs _shapes);

    Vector3d getPosition();
    Orientation3d getOrientation();

    void setPosition(Vector3d _position);
    void setOrientation(Orientation3d _orientation);

private:

    Vector3d m_position;
    Orientation3d m_orientation;
};

typedef std::shared_ptr<UIDataEntryStatic> UIDataEntryStatic_ptr;


} // namespace Data

} // namespace UI

} // namespace LIMoSim

#endif // UIDATAENTRYSTATIC_H
