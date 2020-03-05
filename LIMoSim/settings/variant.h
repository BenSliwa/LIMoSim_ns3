#ifndef LIMOSIM_VARIANT_H
#define LIMOSIM_VARIANT_H

#include <string>

namespace LIMoSim
{

class Variant
{
public:
    Variant();
    Variant(int _data);
    Variant(double _data);
    Variant(const std::string &_data);

    int toInt() const;
    double toDouble() const;
    std::string toString() const;

private:
    std::string m_data;

};

}

#endif // LIMOSIM_VARIANT_H
