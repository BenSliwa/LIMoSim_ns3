#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QSurfaceFormat>
#include <QCommandLineParser>


#include "LIMoSim/mobility/car/strategic/strategicmodel.h"
#include "LIMoSim/mobility/routing/dijkstra.h"
#include "LIMoSim/settings/filehandler.h"
#include "LIMoSim/settings/osm/wgs84.h"
#include "LIMoSim/world/road/road.h"
#include "LIMoSim/world/world.h"

#include "ui/export/epsdocument.h"
#include "ui/opengl/earclipping.h"
//#include "ui/mapmatchingvisualizer.h"
#include "ui/uimanager.h"

#include "standalone/qclidecorators.h"

//#include "app/mapmatching.h"

#include "startsimulation.h"

using namespace LIMoSim;
using namespace Standalone::cliOptions;


int main(int argc, char *argv[])
{
    std::cout << std::endl;

    srand (time(NULL));

    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QGuiApplication app(argc, argv);
    std::cout << std::endl;

    std::cout << QCoreApplication::arguments().join(" ").toStdString() << std::endl;

#ifdef Q_OS_LINUX
    QSurfaceFormat format;// = QSurfaceFormat::defaultFormat();
    format.setVersion(4, 1);
    format.setProfile(QSurfaceFormat::CoreProfile);
    QSurfaceFormat::setDefaultFormat(format);
#endif

    QCommandLineParser parser;
    parser.setApplicationDescription("LIMoSim");
    parser.addHelpOption();
    parser.addVersionOption();

    Standalone::qclidecorators::addGeneralOptions(parser);

//    QCommandLineOption headlessOption( "headless",
//                "Runs LIMoSim without GUI");
//    QCommandLineOption netSimOption(QStringList() << "n" << "netsim",
//                "Choose external network simulator. values: ns3 | standalone", "mode");
//    QCommandLineOption scenarioOption(QStringList() << "s" << "scenario",
//                "Choose simulation scenario");

//    parser.addOption(headlessOption);
//    parser.addOption(netSimOption);
//    parser.addOption(scenarioOption);

    // first parse the args to find out if args processing
    // should take place here on at scenario level.
    // If network simulator option is not set, process the args here.
    // If not skip processing, for it shall be done at one of the lower levels:
    // simulation or scenario.
    parser.parse(QCoreApplication::arguments());
    if (!parser.isSet(netSim())) {
        parser.process(app);
    }

    bool headlessMode = parser.isSet("headless");

    World *world = World::getInstance();
    std::map<std::string, std::string> maps;
    maps["map1"] = "map1.limo";
    maps["tudo"] = "TUDO.osm";
    maps["tudo1"] = "TUDO-larger.osm";
    maps["do-city"] = "City.osm";
    maps["do-sw"] = "dortmund_south_west.osm";

    world->loadMap(maps.at("tudo"));

    std::cout << "init intersections" << std::endl;
    world->filterNodes();
    world->initIntersections();
    world->linkIntersections();
    std::cout << "netsim: " << parser.value(netSim()).toStdString() << std::endl;
    //
    if (!headlessMode) {
        UiManager ui;
        ui.loadQml();

//        MapMatchingVisualizer vis;
        //ui.addVisualizer(&vis);

        return startSimulation(app,
                               !parser.isSet(netSim()) ||
                               parser.value(netSim()) == "standalone"
                               );
    } else {

        return startSimulation(app,
                               !parser.isSet(netSim()) ||
                               parser.value(netSim()) == "standalone"
                               );
    }

}
