#include "animations.h"

#include <iostream>
#include <unistd.h>
#include <QTimer>
#include <QThread>

#include "uidataentry.h"


namespace LIMoSim {

namespace UI {

namespace Data {

Animations::Animations(UIDataEntry *_uiData):
    m_receptionRunning(false),
    m_shapeColor("")
{
}

Animations::~Animations()
{

}

std::string Animations::getShapeColor()
{
    return m_shapeColor;
}


void Animations::triggerReceiving(int _timeout_msec)
{
    m_shapeColor = "white";
    m_receptionRunning = true;
    m_rxEnd =  std::chrono::system_clock::now() + std::chrono::duration<int, std::ratio<1, 1000>> (_timeout_msec);
}

void Animations::triggerTransmitting(int _timeout_msec)
{
    m_shapeColor = "black";
    m_transmissionRunning = true;
    m_txEnd = std::chrono::system_clock::now() + std::chrono::duration<int, std::ratio<1, 1000>> (_timeout_msec);
}

void Animations::updateFlags()
{
    updateReceiving();
    updateTransmitting();

    // Reset values if no animation is running.
    // Done here to avoid order issues
    if (!(m_receptionRunning || m_transmissionRunning)) {
        m_shapeColor = "";
    }
}

void Animations::updateReceiving()
{
    if (std::chrono::system_clock::now() > m_rxEnd) {
        m_receptionRunning = false;
    }
}

void Animations::updateTransmitting()
{
    if (std::chrono::system_clock::now() > m_txEnd) {
        m_transmissionRunning = false;
    }
}

}
}
} // namespace LIMoSim
