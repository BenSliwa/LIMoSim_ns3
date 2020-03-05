#include "antenna.h"
#include "math.h"
#include "LIMoSim/settings/filehandler.h"
#include <iostream>

namespace LIMoSim
{

Antenna::Antenna(const Vector3d &_position, double _phi) :
    m_position(_position),
    m_phi(_phi),
    m_downTilt(0),
    m_maxGain_dB(0),
    m_f_Hz(2.1*pow(10, 9)),
    m_c(3*pow(10, 8)),
    m_alpha(2)
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void Antenna::loadPattern(const std::string &_file)
{
    m_pattern.clear();
    std::vector<std::string> lines = FileHandler::read(_file);
    for(unsigned int i=0; i<lines.size(); i++)
        m_pattern.push_back(atof(lines.at(i).c_str()));

    std::cout << "Antenna::loadPattern " << m_pattern.size() << std::endl;
}

double Antenna::computeRSS(const Vector3d &_position) const
{
    double P_dBm = 43;

    double distance_m = (_position-m_position).norm();
    double L_mW = pow(m_c/(4*3.14*m_f_Hz), 2) * 1/pow(distance_m, m_alpha);
    double L_dB = -mW2dBm(L_mW);
    double rss_mW = dBm2mW(P_dBm) * computeGain(_position);

    return mW2dBm(rss_mW) - L_dB;
}

double Antenna::computeGain(const Vector3d &_position) const
{
    if(m_pattern.size()==0)
        return 1;
    else
    {
        // compute elevation and azimuth
        Vector3d dir = (_position-m_position).normed();
        float azimuth = -Vector3d::computeAngleDifference(m_phi, dir.computePhi());

        double gain = 0;
        int azimuth_ = azimuth + (m_pattern.size()-1)/2;
        if(azimuth_<m_pattern.size())
            gain = m_pattern.at(azimuth_);
        if(gain==0)
            gain = pow(10, -21);
        gain = dBm2mW(m_maxGain_dB) * gain;

        return gain;
    }
}

double Antenna::mW2dBm(double _mW)
{
    return 10 * log10(_mW);
}

double Antenna::dBm2mW(double _dBm)
{
    return pow(10, _dBm/10);
}

Vector3d Antenna::getPosition() const
{
    return m_position;
}

}
