#ifndef LIMOSIM_XMLPARSER_H
#define LIMOSIM_XMLPARSER_H

#include "filehandler.h"
#include "parser.h"
#include "LIMoSim/settings/domelement.h"

namespace LIMoSim
{

class XMLParser : public Parser
{
public:
    XMLParser();

    DOMElement* parse(const std::string &_path);
    DOMElement* parseData(const std::string &_data);

private:
    enum{
        INVALID,
        NAME,
        KEY,
        VALUE,
    };
    int m_state;
    int m_counter;
    bool m_close;
    bool m_meta;
    char m_lastChar;
};

}

#endif // LIMOSIM_XMLPARSER_H
