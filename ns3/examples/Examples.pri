HEADERS += \
	$$PWD/ns3examplescenarios.h

SOURCES += \
        $$PWD/ns3examplescenarios.cc

include (./lte-simple-epc-mobile/LteSimpleEpcMobile.pri)
include (./lte-simple-epc-static/LteSimpleEpcStatic.pri)
include (./lte-simple-static/LteSimpleStatic.pri)
include (./lte-coverage-mobile/LteCoverageMobile.pri)
include (./lte-coverage-static/LteCoverageStatic.pri)
include (./lte-beam-steering/LteBeamSteering.pri)
include (./lte-follower/LteFollower.pri)
include (./lte-static-follower/LteStaticFollower.pri)
include (./lte-aerial-basestation-cluster/LteAerialBasestationCluster.pri)
#include (./parcel-delivery/ParcelDelivery.pri)
include (./wave-follower/WaveFollower.pri)
include (./wifi-reach-mobile/WifiReachMobile.pri)
include (./wifi-reach-static/WifiReachStatic.pri)

equals(ns3, mmw) {
    message("including ns3 mmwave examples in sources")
    include (./mmwave-simple-epc-static/MmwaveSimpleEpcStatic.pri)
    include (./mmwave-car-following/MmwaveCarFollowing.pri)
    include (./mmwave-beam-steering/MmwaveBeamSteering.pri)
}

equals(ns3, v2x) {
    message("including ns3 v2x examples in sources")
    include (./v2x-simple/V2xSimple.pri)
}
