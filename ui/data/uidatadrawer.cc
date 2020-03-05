#include "uidatadrawer.h"

#include <Qt3DCore/QTransform>

#include "uidatamanager.h"
#include "shape.h"
#include "uimanager.h"

// TODO: Move it to world or create interface.
// ns3 specifics should not be include here!
#include "LIMoSim/mobility/external/vehiclenoderegistry.h"

namespace LIMoSim {
namespace UI {
namespace Data {

UIDataDrawer::UIDataDrawer(OpenGLCanvas *_canvas, OpenGLRenderer *_renderer):
    p_canvas(_canvas),
    p_renderer(_renderer)
{

}

void UIDataDrawer::drawVehicles()
{
    auto vehiclesData = UIDataManager::getInstance()->getVehiclesData();
    for (auto it = vehiclesData.begin(); it != vehiclesData.end(); it++) {
        drawVehicle(it->second);
    }
}

void UIDataDrawer::drawStaticNodes()
{
    auto staticNodesData = UIDataManager::getInstance()->getStaticNodesData();
    for (auto it = staticNodesData.begin(); it != staticNodesData.end(); it++) {
        drawStaticNode(it->second);
    }

}

void UIDataDrawer::drawConnectionStatuses()
{
    auto uidm = UIDataManager::getInstance();
    auto connectedNodeIds = uidm->getConnectedNodeIds();

    UIDataEntry_ptr nodeData;
    for (std::string nodeId : connectedNodeIds) {
        auto vnr =  Mobility::External::VehicleNodeRegistry::getInstance();
        nodeData = getNodeUIDataEntry(nodeId);
        if (nodeData)
            drawConnectionStatus(nodeData);
    }
}

void UIDataDrawer::drawConnectionLinks()
{
    Connections connections = UIDataManager::getInstance()->getNodeConnections();

    for (Connection connection : connections) {
        UIDataEntry_ptr nodeData1 = getNodeUIDataEntry(connection.first);
        UIDataEntry_ptr nodeData2 = getNodeUIDataEntry(connection.second);
        if (nodeData1 && nodeData2)
            drawConnectionLink(nodeData1, nodeData2);
    }
}

void UIDataDrawer::drawVehicle(UIDataEntryMobile_ptr _vehicleData)
{
    _vehicleData->processAnimations();
//    Shape_Ptr vehicleShape = _vehicleData->getShape();
    Shape_Ptrs vehicleShapes = _vehicleData->getShapes();

    for (Shape_Ptr shape:vehicleShapes) {
        drawShape(shape, _vehicleData);
    }

//    drawShape(vehicleShape, _vehicleData);
}

void UIDataDrawer::drawStaticNode(UIDataEntryStatic_ptr _staticNodeData)
{
    Shape_Ptr nodeShape = _staticNodeData->getShape();
    drawShape(nodeShape, _staticNodeData);
}

void UIDataDrawer::drawConnectionStatus(UIDataEntry_ptr _nodeData)
{
    Vector3d position = _nodeData->getPosition();
    Vector3d left = Vector3d::fromSphere(90, 90 + _nodeData->getOrientation().z, 2.5/2);
    Vector3d back = left.rotateRight().normed() * 2.5;
    OpenGLShape _s(_nodeData->getShapeColor().c_str());
    _s.addVertex(p_canvas->fromVector(position - left/2 + back/2 + Vector3d(0,0,0.5)));
    p_renderer->drawCircle(_nodeData->getShapeColor().c_str(),
                           p_canvas->fromVector(position - left/2 + back/2 + Vector3d(0,0,0.5)),
                           5,
                           0,
                           false);
}

void UIDataDrawer::drawConnectionLink(UIDataEntry_ptr _nodeData1, UIDataEntry_ptr _nodeData2)
{
    OpenGLShape shape(_nodeData1->getShapeColor().c_str());
    Vector3d pos1 = _nodeData1->getPosition() + Vector3d(0,0,1);
    Vector3d pos2 = _nodeData2->getPosition() + Vector3d(0,0,1);
    Vector3d dir = pos2- pos1;

    Vector3d left = dir.rotateLeft().normed();
    Vector3d right = dir.rotateRight().normed();

    static double linkScaleMax = 5.0;
    static double linkScakeMin = 1.0;
    double linewidth = std::min(linkScaleMax, std::max(linkScakeMin, UiManager::getScale()));
    static double edgeOffset = 11;
    shape.addVertex(p_canvas->fromVector(pos1 + dir.normed() * edgeOffset + left * linewidth));
    shape.addVertex(p_canvas->fromVector(pos1 + dir.normed() * edgeOffset + right * linewidth));
    shape.addVertex(p_canvas->fromVector(pos2 - dir.normed() * edgeOffset + right * linewidth));
    shape.addVertex(p_canvas->fromVector(pos2 - dir.normed() * edgeOffset + left * linewidth));

    p_renderer->drawCube(shape, 0, false);
}

void UIDataDrawer::drawShape(Shape_Ptr _shape, UIDataEntry_ptr _data)
{

    if (_shape->getType() == Cube) {
        ShapeCube_Ptr shapecube = std::dynamic_pointer_cast<ShapeCube>(_shape);
        drawCube(shapecube, _data);
    }
    else if (_shape->getType() == Prism) {
        ShapePrism_Ptr shapeprism = std::dynamic_pointer_cast<ShapePrism>(_shape);
        drawPrism(shapeprism, _data);
    }
    else if (_shape->getType() == Circle) {
        ShapeCircle_Ptr shapecircle = std::dynamic_pointer_cast<ShapeCircle>(_shape);
        drawCircle(shapecircle, _data);
    }
}

void UIDataDrawer::drawCube(ShapeCube_Ptr _shape, UIDataEntry_ptr _data)
{
//    OpenGLShape shape(_data->getShapeColor().c_str());
    OpenGLShape shape(_shape->getColor().c_str());
    Orientation3d orientation = _data->getOrientation() + _shape->getOrientation();
    Vector3d left = Vector3d::fromSphere(90, 90 + orientation.z, _shape->getLeft()/2);
    Vector3d back = left.rotateRight().normed() * _shape->getBack();
    Vector3d position = _data->getPosition() -
            back * _shape->getPosition().x -
            left * _shape->getPosition().y +
            Vector3d(0,0,_shape->getPosition().z);
    shape.addVertex(p_canvas->fromVector(position + left));
    shape.addVertex(p_canvas->fromVector(position - left));
    shape.addVertex(p_canvas->fromVector(position - left + back));
    shape.addVertex(p_canvas->fromVector(position + left + back));

    p_renderer->drawCube(shape, _shape->getHeight(), false);
}

void UIDataDrawer::drawPrism(ShapePrism_Ptr _shape, UIDataEntry_ptr _data)
{
//    OpenGLShape shape(_data->getShapeColor().c_str());
//    Orientation3d orientation = _data->getOrientation() + _shape->getOrientation();
//    Vector3d position = _data->getPosition() + _shape->getPosition();
//    Qt3DCore::QTransform orientationTransformation;
//    orientationTransformation.setRotation(Qt3DCore::QTransform::fromEulerAngles(orientation.roll(), 90 - orientation.pitch(), orientation.yaw()));
//    orientationTransformation.setTranslation(QVector3D(position.x, position.y, position.z));
//    float w =  _shape->getBase(); //2.0;
//    float h = _shape->getHeight();
//    float t = _shape->getDepth(); //0.5;
//    shape.addVertex(orientationTransformation.translation() + (orientationTransformation.rotation() * QVector3D(h, 0, 0)));
//    shape.addVertex(orientationTransformation.translation() + (orientationTransformation.rotation() * QVector3D(0, w/2, 0)));
//    shape.addVertex(orientationTransformation.translation() + (orientationTransformation.rotation() * QVector3D(0, -w/2, 0)));

//    p_renderer->draw3DTriangle(shape, t, false);
}

void UIDataDrawer::drawCircle(ShapeCircle_Ptr _shape, UIDataEntry_ptr _data)
{
//    OpenGLShape shape(_data->getShapeColor().c_str());
//    Orientation3d orientation = _data->getOrientation() + _shape->getOrientation();
//    Vector3d position = _data->getPosition(); // + _shape->getPosition();
//    Qt3DCore::QTransform orientationTransformation;
//    orientationTransformation.setRotation(Qt3DCore::QTransform::fromEulerAngles(orientation.roll(), 90 - orientation.pitch(), orientation.yaw()));
//    orientationTransformation.setTranslation(QVector3D(position.x, position.y, position.z));
//    p_renderer->drawCircle(_shape->getColor().c_str(),
//                           orientationTransformation.translation() + (orientationTransformation.rotation() * p_canvas->fromVector(_shape->getPosition())),
//                           _shape->getRadius(),
//                           0.0,
//                           false);
}

UIDataEntry_ptr UIDataDrawer::getNodeUIDataEntry(std::string _nodeId)
{
    auto uidm = UIDataManager::getInstance();

    UIDataEntry_ptr nodeData = nullptr;

    auto vnr =  Mobility::External::VehicleNodeRegistry::getInstance();
    if (vnr->isNodeRegistered(_nodeId)) {
        nodeData = uidm->getVehicleData(vnr->getVehicleId(_nodeId));
    } else {
        nodeData = uidm->getStaticNodeData(_nodeId);
    }

    return nodeData;
}

}
}
} // namespace LIMoSim
