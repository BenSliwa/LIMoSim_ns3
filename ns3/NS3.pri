
DEFINES+=EXT_NS3

#########################
# NS3 tooling setup
#-----------------------

USER = $$system("echo $USER")
#NS_BUILD_DIR = /home/$$USER/dev/ns3-install-workspace/ns-allinone-3.28/ns-3.28/build
#NS_LIB_DIR =
#NS_LTE_BUILD_DIR = /home/$$USER/dev/ns3-install-workspace/ns-allinone-3.28/ns-3.28/build
NS_LTE_BUILD_DIR = /home/$$USER/dev/ns3-install-workspace/ns-allinone-3.28.1/ns-3.28.1/build
NS_MMW_BUILD_DIR = /home/$$USER/dev/ns3-install-workspace/ns3-mmwave/build
NS_V2X_BUILD_DIR = /home/$$USER/dev/ns3-install-workspace/ns-3-docker/ns-3/ns-3-dev-git/build

NS_LIB_DIR =

# default ns3 (lte without mmw)
NS_BUILD_DIR = $$NS_LTE_BUILD_DIR

equals(ns3, mmw) {
    message("ns3 mmwave build detected")
    DEFINES+=NS_MMW_BUILD
    NS_BUILD_DIR = $$NS_MMW_BUILD_DIR
    NS_LIB_DIR = lib
}

equals(ns3, v2x) {
    message("ns3 v2x build detected")
    DEFINES+=NS_V2X_BUILD
    NS_BUILD_DIR = $$NS_V2X_BUILD_DIR
    NS_LIB_DIR = lib
}

message("using ns3 from $$NS_BUILD_DIR")

INCLUDEPATH += $$NS_BUILD_DIR
LIBS += -L$$NS_BUILD_DIR/$$NS_LIB_DIR

unix {
    SHARED_LIB_FILES = $$files($$NS_BUILD_DIR/$$NS_LIB_DIR/*.so)
    for(FILE, SHARED_LIB_FILES) {
        BASENAME = $$basename(FILE)
        CLEARNAME = $$replace(BASENAME,libns,ns)
        CLEARNAME = $$replace(CLEARNAME,.so,)
        LIBS += -l$$CLEARNAME
    }
#    message("include path: $$INCLUDEPATH")
#    message("libs: $$LIBS")
}
win32 {
    warning("no libs were added for ns3.")
}

#########################




HEADERS += \
    $$PWD/limosimmobilitymodel.h \
    $$PWD/ns3eventscheduler.h \
    $$PWD/ns3utils.h \
    $$PWD/ns3setuphelpers.h \
    $$PWD/callbacks.h

SOURCES += \
    $$PWD/ns3eventscheduler.cc \
    $$PWD/ns3utils.cc \
    $$PWD/limosimmobilitymodel.cc \
    $$PWD/ns3setuphelpers.cc \
    $$PWD/callbacks.cc


include (./examples/Examples.pri)
include (./applications/Applications.pri)
include (./tags/Tags.pri)
include (./modules/Modules.pri)
#include (./strategicmodels/StrategicModels.pri)
#include (./behaviors/Behaviors.pri)
