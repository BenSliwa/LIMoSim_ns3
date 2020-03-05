#include "event.h"
#include "eventhandler.h"

namespace LIMoSim
{

Event::Event(double _timestamp_s, EventHandler *_handler, const std::string &_info) :
    m_timestamp_s(_timestamp_s),
    p_handler(_handler),
    m_info(_info)
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void Event::handle()
{
    p_handler->handleEvent(this);
}

void Event::setTimestamp(double _time_s)
{
    m_timestamp_s = _time_s;
}

void Event::setInfo(const std::string &_info)
{
    m_info = _info;
}

double Event::getTimestamp()
{
    return m_timestamp_s;
}

EventHandler* Event::getHandler()
{
    return p_handler;
}

std::string Event::getInfo()
{
    return m_info;
}


}
