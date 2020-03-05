#include "raytracingheatmap.h"
#include <math.h>
#include "LIMoSim/settings/filehandler.h"
#include <algorithm>

namespace LIMoSim
{

RayTracingHeatMap::RayTracingHeatMap(QObject *_parent) :
    QObject(_parent)
{
    m_colorMap << QColor(0.000000, 0.000000, 143.437500);
    m_colorMap << QColor(0.000000, 0.000000, 159.375000);
    m_colorMap << QColor(0.000000, 0.000000, 175.312500);
    m_colorMap << QColor(0.000000, 0.000000, 191.250000);
    m_colorMap << QColor(0.000000, 0.000000, 207.187500);
    m_colorMap << QColor(0.000000, 0.000000, 223.125000);
    m_colorMap << QColor(0.000000, 0.000000, 239.062500);
    m_colorMap << QColor(0.000000, 0.000000, 255.000000);
    m_colorMap << QColor(0.000000, 15.937500, 255.000000);
    m_colorMap << QColor(0.000000, 31.875000, 255.000000);
    m_colorMap << QColor(0.000000, 47.812500, 255.000000);
    m_colorMap << QColor(0.000000, 63.750000, 255.000000);
    m_colorMap << QColor(0.000000, 79.687500, 255.000000);
    m_colorMap << QColor(0.000000, 95.625000, 255.000000);
    m_colorMap << QColor(0.000000, 111.562500, 255.000000);
    m_colorMap << QColor(0.000000, 127.500000, 255.000000);
    m_colorMap << QColor(0.000000, 143.437500, 255.000000);
    m_colorMap << QColor(0.000000, 159.375000, 255.000000);
    m_colorMap << QColor(0.000000, 175.312500, 255.000000);
    m_colorMap << QColor(0.000000, 191.250000, 255.000000);
    m_colorMap << QColor(0.000000, 207.187500, 255.000000);
    m_colorMap << QColor(0.000000, 223.125000, 255.000000);
    m_colorMap << QColor(0.000000, 239.062500, 255.000000);
    m_colorMap << QColor(0.000000, 255.000000, 255.000000);
    m_colorMap << QColor(15.937500, 255.000000, 239.062500);
    m_colorMap << QColor(31.875000, 255.000000, 223.125000);
    m_colorMap << QColor(47.812500, 255.000000, 207.187500);
    m_colorMap << QColor(63.750000, 255.000000, 191.250000);
    m_colorMap << QColor(79.687500, 255.000000, 175.312500);
    m_colorMap << QColor(95.625000, 255.000000, 159.375000);
    m_colorMap << QColor(111.562500, 255.000000, 143.437500);
    m_colorMap << QColor(127.500000, 255.000000, 127.500000);
    m_colorMap << QColor(143.437500, 255.000000, 111.562500);
    m_colorMap << QColor(159.375000, 255.000000, 95.625000);
    m_colorMap << QColor(175.312500, 255.000000, 79.687500);
    m_colorMap << QColor(191.250000, 255.000000, 63.750000);
    m_colorMap << QColor(207.187500, 255.000000, 47.812500);
    m_colorMap << QColor(223.125000, 255.000000, 31.875000);
    m_colorMap << QColor(239.062500, 255.000000, 15.937500);
    m_colorMap << QColor(255.000000, 255.000000, 0.000000);
    m_colorMap << QColor(255.000000, 239.062500, 0.000000);
    m_colorMap << QColor(255.000000, 223.125000, 0.000000);
    m_colorMap << QColor(255.000000, 207.187500, 0.000000);
    m_colorMap << QColor(255.000000, 191.250000, 0.000000);
    m_colorMap << QColor(255.000000, 175.312500, 0.000000);
    m_colorMap << QColor(255.000000, 159.375000, 0.000000);
    m_colorMap << QColor(255.000000, 143.437500, 0.000000);
    m_colorMap << QColor(255.000000, 127.500000, 0.000000);
    m_colorMap << QColor(255.000000, 111.562500, 0.000000);
    m_colorMap << QColor(255.000000, 95.625000, 0.000000);
    m_colorMap << QColor(255.000000, 79.687500, 0.000000);
    m_colorMap << QColor(255.000000, 63.750000, 0.000000);
    m_colorMap << QColor(255.000000, 47.812500, 0.000000);
    m_colorMap << QColor(255.000000, 31.875000, 0.000000);
    m_colorMap << QColor(255.000000, 15.937500, 0.000000);
    m_colorMap << QColor(255.000000, 0.000000, 0.000000);
    m_colorMap << QColor(239.062500, 0.000000, 0.000000);
    m_colorMap << QColor(223.125000, 0.000000, 0.000000);
    m_colorMap << QColor(207.187500, 0.000000, 0.000000);
    m_colorMap << QColor(191.250000, 0.000000, 0.000000);
    m_colorMap << QColor(175.312500, 0.000000, 0.000000);
    m_colorMap << QColor(159.375000, 0.000000, 0.000000);
    m_colorMap << QColor(143.437500, 0.000000, 0.000000);
    m_colorMap << QColor(127.500000, 0.000000, 0.000000);

    m_cellSize_m = 10;
    m_fontSize = 28;
    m_tickSize = 26;
}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void RayTracingHeatMap::save(const Antenna &_antenna, double _z, const QString &_path)
{
    save(QVector<Antenna>() << _antenna, _z, _path);
}

void RayTracingHeatMap::save(const QVector<Antenna> &_antennas, double _z, const QString &_path)
{
    World *world = World::getInstance();
    Vector3d boxMin = world->getBoxMin();
    Vector3d boxMax = world->getBoxMax();

    int dX = boxMax.x - boxMin.x;
    int dY = boxMax.y - boxMin.y;
    dX = 1 + dX / m_cellSize_m;
    dY = 1 + dY / m_cellSize_m;

    //
    double values[dY][dX];
    for(int i=0; i<_antennas.size(); i++)
    {
        for(int y=0; y<dY; y++)
        {
            qDebug() << "RayTracingHeatMap::save line" << y << " / " << dY;

            for(int x=0; x<dX; x++)
            {
                Vector3d rx(boxMin.x + x*m_cellSize_m + m_cellSize_m/2, boxMin.y + y*m_cellSize_m + m_cellSize_m/2, _z);
                double value = computeRSRP(_antennas.at(i), rx);
                if(i==0)
                    values[y][x] = value;
                else
                    values[y][x] = std::max(values[y][x], value);
            }
        }
    }

    // determine the value range
    double min = std::numeric_limits<double>::max();
    double max = -min;
    for(int y=0; y<dY; y++)
    {
        for(int x=0; x<dX; x++)
        {
            double value = values[y][x];
            if(value<min)
                min = value;
            if(value>max)
                max = value;
        }
    }

                min = -180;
                max = -10;

    // export the eps
    m_offsets = Vector3d(200, 100, 0);
    m_width = dX*m_cellSize_m;
    m_height = dY*m_cellSize_m;
    m_eps.init(m_offsets.x * 2 + m_width, m_offsets.y * 2 + m_height);
    for(int y=0; y<dY; y++)
    {
        for(int x=0; x<dX; x++)
        {
            double value = values[y][x];
            QColor color = getColor(value, min, max);

            if(value==0)
                color = "black";

            drawCell(x, y, color);
        }
    }

    // save .txt file
    QString data;
    QTextStream stream(&data);
    QString path = _path;
    path = path.replace(".eps", ".txt");
    for(int y=0; y<dY; y++)
    {
        for(int x=0; x<dX; x++)
        {
            double value = values[y][x];
            stream << value;
            if(x<dX-1)
                stream << ",";
        }
        if(y<dY-1)
            stream << "\n";
    }
    FileHandler::write(data.toStdString(), path.toStdString());


    drawBuildings();
    drawAxis();
    drawColorBar(min, max);

    m_eps.save(_path);
}

/*************************************
 *           PRIVATE METHODS         *
 ************************************/

void RayTracingHeatMap::drawCell(int _x, int _y, const QColor &_color)
{
    m_eps.drawSquare(QPointF(m_offsets.x + _x*m_cellSize_m + m_cellSize_m/2, m_offsets.y + _y*m_cellSize_m + m_cellSize_m/2), m_cellSize_m, _color);
}

void RayTracingHeatMap::drawCircle(const Vector3d &_center)
{
    World *world = World::getInstance();
    Vector3d boxMin = world->getBoxMin();
    m_eps.drawCircle(QPointF(m_offsets.x + _center.x - boxMin.x, m_offsets.y + _center.y - boxMin.y), 3);
}


void RayTracingHeatMap::drawBuildings()
{
    World *world = World::getInstance();
    Vector3d boxMin = world->getBoxMin();

    std::map<std::string, Building*> buildings = world->getBuildings();
    for(std::map<std::string, Building*>::iterator it=buildings.begin(); it!=buildings.end(); it++)
    {
        Building *building = it->second;
        std::vector<Node*> nodes = building->getNodes();

        m_eps.startPath();
        for(int i=0; i<nodes.size(); i++)
        {
            Vector3d node = nodes.at(i)->getPosition();
            QPointF point(m_offsets.x + node.x - boxMin.x, m_offsets.y + node.y - boxMin.y);

            if(i==0)
                m_eps.moveTo(point);
            else
                m_eps.lineTo(point);
        }
        m_eps.closePath();
        m_eps.setColor("black");
        m_eps.stroke();
    }
}

void RayTracingHeatMap::drawAxis()
{
    // drawBoundingBox
    m_eps.startPath();
    m_eps.moveTo(m_offsets.x, m_offsets.y);
    m_eps.lineTo(m_offsets.x, m_offsets.y+m_height);
    m_eps.lineTo(m_offsets.x+m_width, m_offsets.y+m_height);
    m_eps.lineTo(m_offsets.x+m_width, m_offsets.y);
    m_eps.closePath();
    m_eps.setColor("black");
    m_eps.stroke();

    // draw x-axis
    int steps = 100/m_cellSize_m;
    int l = 5;
    for(int i=0; i<(int)m_width/(m_cellSize_m*steps); i++)
    {
        double x = m_offsets.x + i * m_cellSize_m * steps;
        m_eps.startPath();
        m_eps.moveTo(x, m_offsets.y + l);
        m_eps.lineTo(x, m_offsets.y - l);
        m_eps.closePath();
        m_eps.setColor("black");
        m_eps.stroke();

        m_eps.text(QString::number(i*steps), m_tickSize, x, m_offsets.y-m_tickSize, 0, Qt::AlignHCenter);
    }

    for(int i=0; i<(int)m_height/(m_cellSize_m*steps); i++)
    {
        double y = m_offsets.y + i * m_cellSize_m * steps;
        m_eps.startPath();
        m_eps.moveTo(m_offsets.x + l, y);
        m_eps.lineTo(m_offsets.x - l, y);
        m_eps.closePath();
        m_eps.setColor("black");
        m_eps.stroke();

        m_eps.text(QString::number(i*steps), m_tickSize, m_offsets.x-m_tickSize/2, y, 0, Qt::AlignRight);
    }

    m_eps.text("Longitudinal Cells", m_fontSize, m_offsets.x + m_width/2, m_offsets.y - m_fontSize*2, 0, Qt::AlignHCenter);
    m_eps.text("Latitudinal Cells", m_fontSize, m_offsets.x - m_fontSize*2, m_offsets.y + m_height/2, 90, Qt::AlignHCenter);
}

void RayTracingHeatMap::drawColorBar(double _min, double _max)
{
    double w = 30;
    double h = m_height / 64.0;
    double x = m_offsets.x + m_width + 20;
    double y = m_offsets.y;

    // drawBoundingBox
    m_eps.startPath();
    m_eps.moveTo(x, y);
    m_eps.lineTo(x, y+m_height);
    m_eps.lineTo(x+w, y+m_height);
    m_eps.lineTo(x+w, y);
    m_eps.closePath();
    m_eps.setColor("black");
    m_eps.stroke();


    for(int i=0; i<64; i++)
    {
        m_eps.startPath();
        m_eps.moveTo(x, y + i*h);
        m_eps.lineTo(x, y + (i+1)*h);
        m_eps.lineTo(x + w, y + (i+1)*h);
        m_eps.lineTo(x + w, y + i*h);

        m_eps.closePath();
        m_eps.setColor(m_colorMap.at(i));
        m_eps.fill();
    }

    int steps = 4;
    double stepHeight = m_height/(double)steps;
    int l = 5;
    for(int i=0; i<steps+1; i++)
    {
        double y = m_offsets.y + i * stepHeight;
        m_eps.startPath();
        m_eps.moveTo(x + w + l, y);
        m_eps.lineTo(x + w - l, y);
        m_eps.closePath();
        m_eps.setColor("black");
        m_eps.stroke();

        int value = _min + i * (_max-_min) / (steps);
        m_eps.text(QString::number(value), m_tickSize, x + w + 2 * l, y, 0, Qt::AlignLeft);
    }


    m_eps.text("RSRP [dBm]", m_fontSize, x + w + m_fontSize*3, m_offsets.y + m_height/2, -90, Qt::AlignHCenter);
}

double RayTracingHeatMap::computeRSRP(const Antenna &_antenna, const Vector3d &_rx)
{
    Raytracing rayTracing;
    double beta = 2.4;
    double gamma = 0.63;

    RayTrace trace = rayTracing.trace(_antenna.getPosition(), _rx);
    double L_obs = trace.intersections.size() * beta + trace.attenuated_m * gamma;
    double rss = _antenna.computeRSS(_rx);
    double rsrp = rss - L_obs;

    return rsrp;
}

QColor RayTracingHeatMap::getColor(double _value, double _min, double _max)
{
    int index = (_value-_min) * 63.0 / (_max-_min);
    if(index<0)
        index = 0;
    if(index>63)
        index = 63;

    return m_colorMap.at(index);
}

}
