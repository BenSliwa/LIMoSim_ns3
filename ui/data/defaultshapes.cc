#include "defaultshapes.h"

#include "LIMoSim/mobility/vehicle.h"

namespace LIMoSim {
namespace UI {
namespace Data {

Shape_Ptrs defaultCarShape() {
    return Shape_Ptrs {std::make_shared<ShapeCube>(ShapeCube(([]() {return 3.5;}),
                                                             ([](){return 2;}),
                                                             1.5,
                                                             Vector3d(-0.2,-0.15,0.5))),
                std::make_shared<ShapeCube>(ShapeCube(([]() {return 5;}),
                                                      ([](){return 3;}),
                                                      2,
                                                      Vector3d(0.5,0,0.5))),
                std::make_shared<ShapeCube>(ShapeCube(([]() {return 0.75;}),
                                                      ([](){return 2.5;}),
                                                      0.7,
                                                      "black",
                                                      Vector3d(3,0,-0.0))),
                std::make_shared<ShapeCube>(ShapeCube(([]() {return 0.75;}),
                                                      ([](){return 2.5;}),
                                                      0.7,
                                                      "black",
                                                      Vector3d(-2,0,-0.0)))
    };
}

Shape_Ptrs defaultUAVShape() {
    //    return Shape_Ptrs {std::make_shared<ShapePrism>(ShapePrism())};
    return Shape_Ptrs {std::make_shared<ShapeCube>(ShapeCube(([] () {return 2.0;}),
                                                             ([] () {return 2.0;}),
                                                             0.5,
                                                             "blue",
                                                             Vector3d(0.5,0.5))),
                //                std::make_shared<ShapePrism>(ShapePrism(2,0.5,3)),
                std::make_shared<ShapeCircle>(ShapeCircle(0.5, "yellow", Vector3d(1.0,1.0))),
                std::make_shared<ShapeCircle>(ShapeCircle(0.5, "yellow", Vector3d(1.0,-1.0))),
                std::make_shared<ShapeCircle>(ShapeCircle(0.5, "yellow", Vector3d(-1.0,1.0))),
                std::make_shared<ShapeCircle>(ShapeCircle(0.5, "yellow", Vector3d(-1.0,-1.0)))
    };
}

Shape_Ptrs defaultVehicleShape(Vehicle *_vehicle) {
    Shape_Ptrs shapes;
    if (_vehicle->getType() == "Car") {
        shapes = defaultCarShape();
    } else if (_vehicle->getType() == "UAV") {
        shapes = defaultUAVShape();
    } else {
        shapes = defaultUAVShape();
    }
    return shapes;
}

Shape_Ptrs defaultGateShape(std::string _color) {
    Shape_Ptrs shapes;
    return Shape_Ptrs {std::make_shared<ShapeCube>(ShapeCube(
        [](){return 3;},
        []() {return 3;},
        50.1,
        _color,
        Vector3d(0,0,5),
        Orientation3d(-90)
    ))};
}



}
}
}
