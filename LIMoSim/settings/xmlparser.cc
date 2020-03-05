#include "xmlparser.h"
#include "LIMoSim/settings/domdefinitions.h"

namespace LIMoSim
{

XMLParser::XMLParser() : Parser(),
    m_state(INVALID),
    m_counter(0)
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

DOMElement* XMLParser::parse(const std::string &_path)
{
    std::string data = m_fileHandler.listToString(m_fileHandler.read(_path));
    DOMElement* parsedData = parseData(data);

    return parsedData;
}

DOMElement* XMLParser::parseData(const std::string &_data)
{
    DOMElement *document = 0;
    DOMElement *node = document;

    //
    std::string buffer = "";
    std::string field = "";

    std::string key = "";
    std::string value = "";
    for(unsigned int i=0; i<_data.size(); i++)
    {
        char c = _data.at(i);

        if(c=='<')
        {
            // here: buffer = space between nodes
            {
                // handle the buffer
            }

            //
            m_close = false;
            buffer = "";
            buffer += c;
            field = "";
            m_counter = 0;

            m_state = NAME;
        }
        else if(c=='>')
        {
            buffer += c;

            // tag ended

            // go one layer up if the tag is closed
            if(m_close)
            {
                if(node && node->parentNode)
                    node->parentNode->appendChild(node);

                //
                if(node)
                    node = (DOMElement*)node->parentNode;
            }
            else
            {
                if(node && node->tagName=="")
                    node->tagName = field;
            }

            //std::cout << buffer << std::endl;
            buffer = "";
            m_state = INVALID;
        }
        else // inside a tag
        {
            if(m_state==NAME)
            {
                if(m_lastChar=='<' && c!='/')
                {
                    //std::cout << "down" << std::endl;

                    // go one layer down
                    DOMElement *parent = node;
                    node = new DOMElement();
                    node->parentNode = parent;

                    if(!document && c!='?')
                        document = node;
                }

                if(c==' ')
                {
                    // name ended
                    //std::cout << "name field: " << field << std::endl;

                    node->tagName = field;

                    field = "";
                    m_state = KEY;
                    key = "";
                    value = "";
                }
                else if(c=='/' || c=='?')
                    m_close = true;
                else
                    field += c;
            }
            else if(m_state==KEY)
            {
                if(c==' ') // ignore space between attributes
                {

                }
                else if(c=='/' || c=='?') // closing marker for inline
                {
                    m_close = true;
                }
                else if(c=='=')
                {
                    m_state = VALUE;
                }
                else
                    key += c;
            }
            else if(m_state==VALUE)
            {
                if(c=='\"' || c=='\'')
                {
                    m_counter++;
                    if(m_counter%2==0)
                    {
                        node->setAttribute(key, value);

                        m_state = KEY;

                        // reset
                        key = "";
                        value = "";
                    }

                }
                else if(m_counter%2==1)
                    value += c;
            }


            buffer += c;
        }

        m_lastChar = c;
    }

    return document;
}

}

