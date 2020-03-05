#include "shape.h"

namespace OpenGL
{

Shape::Shape(const QColor &_color) :
    color(_color)
{

}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/


QVector<float> Shape::getData() const
{
    QVector<float> data;
    QVector<float> c = getColorData(color);

    for(int i=0; i<vertices.size(); i++)
    {
        Vertex vertex = vertices.at(i);
        data << vertex.x << vertex.y << vertex.z;
        data << c;
    }


    return data;
}

QVector<float> Shape::getColorData(const QColor &_color) const
{
    QVector<float> data;

    int r, g, b;
    _color.getRgb(&r,&g,&b);
    data << r/255.0 << g/255.0 << b/255.0;

    return data;
}

QString Shape::toString() const
{
    QString data;

    for(int i=0; i<vertices.size(); i++)
    {
        data += vertices.at(i).toString();
        data += "\n";
    }

    return data;
}

}
