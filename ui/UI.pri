include(export/Export.pri)
include(qml/QML.pri)
include(opengl/OpenGL.pri)
include(standalone/Standalone.pri)
include(data/Data.pri)

HEADERS += \
    $$PWD/uimanager.h \
    $$PWD/canvas.h \
    $$PWD/uimanagerservice.h \
    $$PWD/visualizer.h

SOURCES += \
    $$PWD/uimanager.cc \
    $$PWD/canvas.cc \
    $$PWD/uimanagerservice.cpp \
    $$PWD/visualizer.cc
