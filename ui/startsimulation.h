#ifndef STARTSIMULATION_H
#define STARTSIMULATION_H

#include <QGuiApplication>


namespace LIMoSim {

int standaloneSimulation(QGuiApplication& _app);

#ifdef EXT_NS3
void ns3SimulationScript();
int ns3Simulation(QGuiApplication& _app);
#endif

int startSimulation(QGuiApplication& _app, bool _standalone = true);

}
#endif // STARTSIMULATION_H
