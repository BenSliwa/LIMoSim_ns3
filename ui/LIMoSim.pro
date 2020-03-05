QT += qml quick
CONFIG += c++11
CONFIG += c++17

SOURCES += main.cc \
    startsimulation.cc

# Additional import path used to resolve QML modules in Qt Creator's code model
QML_IMPORT_PATH =

INCLUDEPATH += $$PWD/../LIMoSim
INCLUDEPATH += $$PWD/../ # project view from core

include(../LIMoSim/LIMoSim.pri)
include(UI.pri)
include (../standalone/Standalone.pri)
include(../demo/Demo.pri)

contains(exts, ns3) {
    include (../ns3/NS3.pri)
}

HEADERS += \
    startsimulation.h \
    ns3simulationscript.h

message("copying resources from $$PWD/../misc/resources to $$OUT_PWD")
@
QMAKE_POST_LINK += $$quote($(COPY) $$PWD/../misc/resources/* $$OUT_PWD/)
@
