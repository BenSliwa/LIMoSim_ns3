#ifndef LIMOSIM_EPSDOCUMENT_H
#define LIMOSIM_EPSDOCUMENT_H

#include <QObject>
#include <QDebug>
#include <QColor>
#include <QPolygonF>

namespace LIMoSim
{

struct LineStyle
{
    double width;
    QColor color;
};

class EpsDocument
{
public:
    EpsDocument();

    void init(double _width, double _height);

    //
    void drawCircle(const QPointF &_position, double _radius, const QColor &_color = "blue");
    void drawSquare(const QPointF &_center, double _width, const QColor &_color = "blue");

    void text(const QString &_text, double _size, double _x, double _y, int _rotation, int _horizontal);

    // primitives
    void startPath();
    void closePath();
    void moveTo(const QPointF &_position);
    void moveTo(double _x, double _y);
    void lineTo(const QPointF &_position);
    void lineTo(double _x, double _y);
    void setColor(const QColor &_color);
    void setLineWidth(double _width);
    void setLineStyle(int _style);
    void fill();
    void stroke();

    void save(const QString &_file);
    void alignText(int _horizontal, int _vertical);

private:
    QString m_data;


};

}

#endif // LIMOSIM_EPSDOCUMENT_H
