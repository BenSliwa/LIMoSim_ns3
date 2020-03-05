#include "qclidecorators.h"

#include <QCommandLineParser>

namespace LIMoSim {
namespace Standalone {

namespace cliOptions {

QCommandLineOption &headless()
{
    static QCommandLineOption headlessOption(
                "headless",
                "Runs LIMoSim without GUI");
    return headlessOption;
}

QCommandLineOption &listScenarios()
{
    static QCommandLineOption listOption(
                QStringList() << "l" << "list",
                "list available simulation scenarios");
    return listOption;
}

QCommandLineOption &netSim()
{
    static QCommandLineOption netSimOption(
                QStringList() << "n" << "netsim",
                "Choose external network simulator. values: ns3 | standalone", "mode");
    return netSimOption;
}

QCommandLineOption &scenario()
{
    static QCommandLineOption scenarioOption(
                QStringList() << "s" << "scenario",
                "Choose simulation scenario", "name");
    return scenarioOption;
}



} // namespace cliOptions

namespace qclidecorators {

void addGeneralOptions(QCommandLineParser &_parser)
{

    _parser.addOption(cliOptions::headless());
    _parser.addOption(cliOptions::netSim());
}

void addSimulationOptions(QCommandLineParser &_parser)
{
    _parser.addOption(cliOptions::scenario());
    _parser.addOption(cliOptions::listScenarios());
}

}

// namespace qclidecorators
} // namespace Standalone
} // namespace LIMoSim
