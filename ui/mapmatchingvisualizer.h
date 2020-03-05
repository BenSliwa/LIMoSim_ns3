#ifndef LIMOSIM_MAPMATCHINGVISUALIZER_H
#define LIMOSIM_MAPMATCHINGVISUALIZER_H

#include "visualizer.h"
#include "app/mobilitytrace.h"
#include "app/mapmatching.h"

namespace LIMoSim
{

class MapMatchingVisualizer : public Visualizer
{
public:
    MapMatchingVisualizer();

    //
    void update(UiManager *_ui);

private:
    MobilityTrace m_trace;
};


}

#endif // LIMOSIM_MAPMATCHINGVISUALIZER_H
