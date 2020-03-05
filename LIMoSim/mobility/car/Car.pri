HEADERS += \
    $$PWD/car.h

SOURCES += \
    $$PWD/car.cc

include(awareness/Awareness.pri)
include(following/FollowingModel.pri)
include(strategic/StrategicModel.pri)
include(prediction/Prediction.pri)
