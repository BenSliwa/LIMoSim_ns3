HEADERS += \
    $$PWD/xmlparser.h \
    $$PWD/filehandler.h \
    $$PWD/domelement.h \
    $$PWD/variant.h \
    $$PWD/domdefinitions.h \
    $$PWD/domnode.h \
    $$PWD/parser.h

SOURCES += \
    $$PWD/xmlparser.cc \
    $$PWD/filehandler.cc \
    $$PWD/domelement.cc \
    $$PWD/variant.cc \
    $$PWD/domnode.cc \
    $$PWD/parser.cc

include(osm/OSM.pri)
