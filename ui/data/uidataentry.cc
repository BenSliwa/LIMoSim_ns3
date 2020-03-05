#include "uidataentry.h"

namespace LIMoSim {

namespace UI {

namespace Data {
UIDataEntry::UIDataEntry(std::string _id, UIDataEntryType _type, Shape_Ptr _shape):
    m_id(_id),
    m_type(_type),
    m_shape(_shape),
    m_animations( new Animations(this))
{
    m_shapes.push_back(_shape);
}

UIDataEntry::UIDataEntry(std::string _id, UIDataEntryType _type, Shape_Ptrs _shapes):
    m_id(_id),
    m_type(_type),
    m_shape(_shapes.at(0)),
    m_shapes(_shapes),
    m_animations( new Animations(this))
{

}

UIDataEntry::~UIDataEntry()
{
}

std::string UIDataEntry::getId()
{
    return m_id;
}

UIDataEntryType UIDataEntry::getType()
{
    return m_type;
}

Shape_Ptr UIDataEntry::getShape()
{
    return m_shapes.at(0);
//    return m_shape;
}

Shape_Ptrs UIDataEntry::getShapes()
{
    return m_shapes;
}

std::string UIDataEntry::getShapeColor()
{
    if (m_animations->getShapeColor().empty()) {
        return m_shapes.at(0)->getColor();
//        return m_shape->getColor();
    } else {
        return m_animations->getShapeColor();
    }
}

void UIDataEntry::setShape(Shape_Ptr _shape)
{
    m_shapes[0] = _shape;
    m_shape = _shape;
}

void UIDataEntry::setShapes(Shape_Ptrs _shapes)
{
    m_shapes = _shapes;
}

void UIDataEntry::animateReception()
{
    m_animations->triggerReceiving(100);
    return;
}

void UIDataEntry::animateTransmission()
{
    m_animations->triggerTransmitting(100);
    return;
}

void UIDataEntry::processAnimations()
{
    m_animations->updateFlags();
}

}
}
}
