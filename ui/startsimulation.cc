#include "startsimulation.h"

#include <QtConcurrent/QtConcurrent>
#include<QFutureWatcher>

#include <time.h>
#include <functional>

#include "LIMoSim/simulation/simulation.h"
#include "LIMoSim/world/vehiclemanager.h"
#include "ui/standalone/standaloneeventscheduler.h"
#include "ui/data/uidatamanager.h"
#include "ui/uimanagerservice.h"

#ifdef EXT_NS3
#include "ns3simulationscript.h"
// NS3
#include "ns3/ns3setuphelpers.h"
#include "ns3/ns3eventscheduler.h"
#endif

#include "standalone/scenarios.h"

namespace LIMoSim {

int standaloneSimulation(QGuiApplication &_app) {

    StandaloneEventScheduler *scheduler = new StandaloneEventScheduler();
    Simulation *simulation = Simulation::getInstance(scheduler);

    // connect vehicle manager to ui data manager
    VehicleManager::getInstance()->setUIDataHandler(
                UI::Data::UIDataManager::getInstance()
                );
    // enable exchange of mobility data among vehicles
    // by connecting the vehicle manager and local vehicle manager instances
    VehicleManager::getInstance()->enableMobilityBroadcastHelper();

    Standalone::helpers::configureScenario();

    simulation->run();

    return _app.exec();
}


#ifdef EXT_NS3

int ns3Simulation(QGuiApplication &_app) {

    using namespace ns3;
    using namespace NS3;

    EventScheduler *scheduler = new NS3EventScheduler();
    Simulation *simulation = Simulation::getInstance(scheduler);

    setupNS3Globals();

    // connect vehicle manager to ui data manager
    VehicleManager::getInstance()->setUIDataHandler(
                UI::Data::UIDataManager::getInstance()
                );

    std::function<void()> ns3Script = [&simulation]() {
        ns3SimulationScript();
//        wireUpLIMoSimAndStartSimulation();
//        createLIMoSimVehicles();
//        registerStaticNodes();
//        simulation->run();
    };

    QFutureWatcher<void>* ns3ScriptWatcher = new QFutureWatcher<void>();
    QObject::connect(ns3ScriptWatcher, &QFutureWatcher<void>::finished, [&_app]() { _app.quit(); });
    QObject::connect(&_app, &QGuiApplication::aboutToQuit , [&simulation]() {
        simulation->stop();
    });
    ns3ScriptWatcher->setFuture(QtConcurrent::run(ns3Script));


    while (!LIMoSim::ui::UiManagerService::getInstance()->getUiDataInitialized()) {
        sleep(1);
    }

    return _app.exec();
}
#endif

int startSimulation(QGuiApplication &_app, bool _standalone) {
    // runtime switch for simulation mode
    bool standalone = _standalone;

#ifdef EXT_NS3
    if (standalone) {
        return standaloneSimulation(_app);
    } else {
        return ns3Simulation(_app);
    }
#else
    return standaloneSimulation(_app);
#endif
}

}
