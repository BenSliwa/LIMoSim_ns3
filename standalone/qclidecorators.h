#ifndef QCLIDECORATORS_H
#define QCLIDECORATORS_H

class QCommandLineParser;
class QCommandLineOption;

namespace LIMoSim {
namespace Standalone {

namespace cliOptions {

QCommandLineOption & headless();
QCommandLineOption & listScenarios();
QCommandLineOption & netSim();
QCommandLineOption & scenario();

} // namespace cliOptions

namespace qclidecorators {

void addGeneralOptions(QCommandLineParser &_parser);
void addSimulationOptions(QCommandLineParser &_parser);

} // namespace qclidecorators
} // namespace Standalone
} // namespace LIMoSim

#endif // QCLIDECORATORS_H
