#ifndef RAYTRACINGHEATMAP_H
#define RAYTRACINGHEATMAP_H

#include <QObject>
#include <QDebug>
#include "LIMoSim/world/world.h"
#include "LIMoSim/world/raytracing/raytracing.h"
#include "LIMoSim/world/raytracing/antenna.h"
#include "epsdocument.h"

namespace LIMoSim
{

class RayTracingHeatMap : public QObject
{
    Q_OBJECT
public:
    RayTracingHeatMap(QObject *_parent = 0);

    void save(const Antenna &_antenna, double _z, const QString &_path);
    void save(const QVector<Antenna> &_antennas, double _z, const QString &_path);

public:
    void drawCell(int _x, int _y, const QColor &_color);
    void drawCircle(const Vector3d &_center);

    void drawBuildings();
    void drawAxis();
    void drawColorBar(double _min, double _max);

    double computeRSRP(const Antenna &_antenna, const Vector3d &_rx);
    QColor getColor(double _value, double _min, double _max);


private:
    EpsDocument m_eps;
    QVector<QColor> m_colorMap;
    Vector3d m_offsets;
    double m_width;
    double m_height;
    double m_cellSize_m;

    double m_fontSize;
    double m_tickSize;
};

}

#endif // RAYTRACINGHEATMAP_H
