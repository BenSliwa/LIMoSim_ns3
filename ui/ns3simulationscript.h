#ifndef NS3SIMULATIONSCRIPT_H
#define NS3SIMULATIONSCRIPT_H

#include "standalone/scenarios.h"

// ns3
#include <ns3/log.h>

// LIMoSim ns3
#include "ns3/examples/ns3examplescenarios.h"
#include "ns3/ns3setuphelpers.h"
#include <unistd.h>

#include "standalone/scenarioregistry.h"
#include "standalone/qclidecorators.h"

#include <QCommandLineParser>


namespace LIMoSim {

using namespace ns3;

inline void ns3SimulationScript() {
    LogComponentEnable("NS3SetupHelpers", LOG_LEVEL_INFO);
    NS3::setDeterministicCarRouteEnabled(false);

    QCommandLineParser parser;
    parser.setApplicationDescription("LIMoSim standalone");
    parser.addHelpOption();
    parser.addVersionOption();

    Standalone::qclidecorators::addGeneralOptions(parser);
    Standalone::qclidecorators::addSimulationOptions(parser);

    NS3::helpers::loadScenarioRegistry();

    parser.parse(QCoreApplication::arguments());
    if (!parser.isSet(Standalone::cliOptions::scenario())) {
        if (parser.isSet("help")) {
            std::cout << parser.helpText().toStdString() << std::endl;
        }
    }

    if (parser.isSet(Standalone::cliOptions::listScenarios())) {
        NS3::helpers::printAvailableScenarios();
        return;
    }

    if (parser.isSet(Standalone::cliOptions::scenario())) {
        Scenario s = ScenarioRegistry::getInstance()->getScenario(parser.value(Standalone::cliOptions::scenario()).toStdString());
        if (s) {

            /* Your ns3 simulation script goes here*/
            return s();
        } else {
            std::cerr << "scenario " << parser.value(Standalone::cliOptions::scenario()).toStdString() << "was not found!" << std::endl;
        }
    } else {
        // In case no simulation scenario was specified on Cli
        // the old script mechanism kicks in.
        // A specific scenario can be choosen for the whole build.

        /* Your ns3 simulation script goes here*/
        NS3::Examples::lteAerialBasestationCluster();
    }



}
}

#endif // NS3SCRIPT_H
