include(car/Car.pri)
include(routing/Routing.pri)
include(uav/UAV.pri)
include(external/External.pri)
#include(deliverytruck/DeliveryTruck.pri)

HEADERS += \
    $$PWD/vehicle.h \
    $$PWD/mobilitydata.h \
    $$PWD/mobilitymodel.h

SOURCES += \
    $$PWD/vehicle.cc \
    $$PWD/mobilitydata.cc \
    $$PWD/mobilitymodel.cc
