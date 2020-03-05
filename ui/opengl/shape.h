#ifndef OPENGL_SHAPE_H
#define OPENGL_SHAPE_H

#include <QColor>
#include <QVector>
#include <QDebug>
#include "vertex.h"

namespace OpenGL
{

class Shape
{
public:
    Shape(const QColor &_color = "white");

    virtual QVector<float> getData() const;

    QVector<float> getColorData(const QColor &_color) const;

    QString toString() const;

public:
    QList<Vertex> vertices;
    QColor color;
};

}


#endif // OPENGL_SHAPE_H
