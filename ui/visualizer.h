#ifndef LIMOSIM_VISUALIZER_H
#define LIMOSIM_VISUALIZER_H

namespace LIMoSim
{

class UiManager;

class Visualizer
{
public:
    Visualizer();

    virtual void update(UiManager *_ui) = 0;
};


}


#endif // LIMOSIM_VISUALIZER_H
