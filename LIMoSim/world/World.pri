HEADERS += \
    $$PWD/area.h \
    $$PWD/node.h \
    $$PWD/way.h \
    $$PWD/worldobject.h \
    $$PWD/world.h \
    $$PWD/vector3d.h \
    $$PWD/vehiclemanager.h \
    $$PWD/building.h \
    $$PWD/orientation3d.h \
    $$PWD/localvehiclemanager.h \
    $$PWD/matrix3d.h \
    $$PWD/uidata.h \
    $$PWD/worldtypes.h \
    $$PWD/worldutils.h

SOURCES += \
    $$PWD/area.cc \
    $$PWD/localvehiclemanager.cc \
    $$PWD/node.cc \
    $$PWD/orientation3d.cc \
    $$PWD/way.cc \
    $$PWD/world.cc \
    $$PWD/worldobject.cc \
    $$PWD/vector3d.cc \
    $$PWD/vehiclemanager.cc \
    $$PWD/building.cc \
    $$PWD/matrix3d.cc \
    $$PWD/worldutils.cc

include(road/Road.pri)
include(raytracing/Raytracing.pri)
