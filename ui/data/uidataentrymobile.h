#ifndef UIDATAENTRYMOBILE_H
#define UIDATAENTRYMOBILE_H

#include "uidataentry.h"
#include <memory>
#include <functional>

#include "LIMoSim/world/vector3d.h"
#include "LIMoSim/world/orientation3d.h"

namespace LIMoSim {

namespace UI {

namespace Data {

class UIDataEntryMobile : public UIDataEntry
{
public:
    UIDataEntryMobile(
            std::string _id,
            std::function<Vector3d(void)> _position,
            std::function<Orientation3d(void)> _orientation);

    UIDataEntryMobile(
            std::string _id,
            std::function<Vector3d(void)> _position,
            std::function<Orientation3d(void)> _orientation,
            Shape_Ptr _shape);

    UIDataEntryMobile(
            std::string _id,
            std::function<Vector3d(void)> _position,
            std::function<Orientation3d(void)> _orientation,
            Shape_Ptrs _shapes);

    Vector3d getPosition();
    Orientation3d getOrientation();

private:
    std::function <Vector3d(void)> m_position;
    std::function<Vector3d(void)> m_orientation;
};


typedef std::shared_ptr<UIDataEntryMobile> UIDataEntryMobile_ptr;


} // namespace Data

} // namespace UI

} // namespace LIMoSim

#endif // UIDATAENTRYMOBILE_H
