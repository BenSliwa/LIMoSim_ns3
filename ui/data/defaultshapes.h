#ifndef DEFAULTSHAPES_H
#define DEFAULTSHAPES_H

#include "shapes.h"

namespace LIMoSim {

class Vehicle;

namespace UI {

namespace Data {

Shape_Ptrs defaultCarShape();

Shape_Ptrs defaultUAVShape();

Shape_Ptrs defaultVehicleShape(Vehicle *_vehicle);

Shape_Ptrs defaultGateShape(std::string _color="green");

}

}

}



#endif // DEFAULTSHAPES_H
