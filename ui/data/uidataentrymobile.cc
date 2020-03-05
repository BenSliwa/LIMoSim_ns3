#include "uidataentrymobile.h"

#include "shapeprism.h"

namespace LIMoSim {

namespace UI {

namespace Data {

UIDataEntryMobile::UIDataEntryMobile(
        std::string _id,
        std::function <Vector3d(void)>_position,
        std::function<Orientation3d(void)> _orientation):
    UIDataEntry(_id, MOBILE, std::make_shared<ShapePrism>(ShapePrism())),
    m_position(_position),
    m_orientation(_orientation)
{
}

UIDataEntryMobile::UIDataEntryMobile(
        std::string _id,
        std::function <Vector3d(void)>_position,
        std::function<Orientation3d(void)> _orientation,
        Shape_Ptr _shape):
    UIDataEntry(_id, MOBILE, _shape),
    m_position(_position),
    m_orientation(_orientation)
{
}

UIDataEntryMobile::UIDataEntryMobile(std::string _id,
        std::function <Vector3d(void)>_position,
        std::function<Orientation3d(void)> _orientation,
        Shape_Ptrs _shapes):
    UIDataEntry(_id, MOBILE, _shapes),
    m_position(_position),
    m_orientation(_orientation)
{
}


Vector3d UIDataEntryMobile::getPosition()
{
    return m_position();
}

Orientation3d UIDataEntryMobile::getOrientation()
{
    return m_orientation();
}


} // namespace Data

} // namespace UI

} // namespace LIMoSim
