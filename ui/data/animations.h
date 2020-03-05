#ifndef ANIMATIONS_H
#define ANIMATIONS_H

#include <chrono>
#include <string>

namespace LIMoSim {

namespace UI {

namespace Data {

class UIDataEntry;

class Animations {
public:
    Animations(UIDataEntry* _uiData);
    virtual ~Animations();

    std::string getShapeColor();

    //*****************************************************************//
    // Animationstriggern                                              //
    //*****************************************************************//
    void triggerReceiving(int _timeout_msec);
    void triggerTransmitting(int _timeout_msec);



    void updateFlags();
    void updateReceiving();
    void updateTransmitting();

private:

    bool m_receptionRunning;
    std::chrono::system_clock::time_point m_rxEnd;

    bool m_transmissionRunning;
    std::chrono::system_clock::time_point m_txEnd;

    std::string m_shapeColor;
};

}
}
} // namespace LIMoSim

#endif // ANIMATIONS_H
