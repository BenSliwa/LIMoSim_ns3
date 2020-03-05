#ifndef UIDATADRAWER_H
#define UIDATADRAWER_H

#include "uidataentry.h"
#include "uidataentrymobile.h"
#include "uidataentrystatic.h"

#include "shapecube.h"
#include "shapeprism.h"
#include "shapecircle.h"
#include "ui/opengl/openglrenderer.h"
#include "ui/opengl/openglcanvas.h"

namespace LIMoSim {
namespace UI {
namespace Data {

using namespace OpenGL;

class UIDataDrawer
{
public:
    UIDataDrawer(OpenGLCanvas *_canvas, OpenGLRenderer *_renderer);

    void drawVehicles();
    void drawStaticNodes();
    void drawConnectionStatuses();
    void drawConnectionLinks();

    void drawVehicle(UIDataEntryMobile_ptr _vehicleData);
    void drawStaticNode(UIDataEntryStatic_ptr _staticNodeData);
    void drawConnectionStatus(UIDataEntry_ptr _nodeData);
    void drawConnectionLink(UIDataEntry_ptr _nodeData1, UIDataEntry_ptr _nodeData2);
    void drawShape(Shape_Ptr _shape, UIDataEntry_ptr _data);

private:
    OpenGL::OpenGLCanvas *p_canvas;
    OpenGLRenderer *p_renderer;

    // Shape drawing interface
    void drawCube(ShapeCube_Ptr _shape, UIDataEntry_ptr _data);
    void drawPrism(ShapePrism_Ptr _shape, UIDataEntry_ptr _data);
    void drawCircle(ShapeCircle_Ptr _shape, UIDataEntry_ptr _data);

    UIDataEntry_ptr getNodeUIDataEntry(std::string _nodeId);

};

}
}
} // namespace LIMoSim

#endif // UIDATADRAWER_H
