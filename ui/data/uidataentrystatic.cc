#include "uidataentrystatic.h"
#include "shapecube.h"
namespace LIMoSim {

namespace UI {

namespace Data {

UIDataEntryStatic::UIDataEntryStatic(std::string _id):
    UIDataEntry(_id, STATIC, std::make_shared<ShapeCube>(ShapeCube("pink")))
{

}

UIDataEntryStatic::UIDataEntryStatic(std::string _id, Vector3d _position, Orientation3d _orientation):
    UIDataEntry(_id, STATIC, std::make_shared<ShapeCube>(ShapeCube("pink"))),
    m_position(_position),
    m_orientation(_orientation)
{

}

UIDataEntryStatic::UIDataEntryStatic(std::string _id,
                                     Vector3d _position,
                                     Orientation3d _orientation,
                                     Shape_Ptrs _shapes):
    UIDataEntry(_id, STATIC, _shapes),
    m_position(_position),
    m_orientation(_orientation)
{

}

Vector3d UIDataEntryStatic::getPosition()
{
    return m_position;
}

Orientation3d UIDataEntryStatic::getOrientation()
{
    return m_orientation;
}

void UIDataEntryStatic::setPosition(Vector3d _position)
{
    m_position = _position;
}

void UIDataEntryStatic::setOrientation(Orientation3d _orientation)
{
    m_orientation = _orientation;
}


} // namespace Data

} // namespace UI

} // namespace LIMoSim
