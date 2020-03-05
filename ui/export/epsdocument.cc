#include "epsdocument.h"
#include "LIMoSim/settings/filehandler.h"
#include <QDateTime>

namespace LIMoSim
{

EpsDocument::EpsDocument()
{
}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void EpsDocument::init(double _width, double _height)
{
    m_data = "";

    // define the EPS header
    m_data += "%!PS-Adobe-3.0 EPSF-3.0\n";
    m_data += "%%CreationDate:" + QDateTime::currentDateTime().toString("yyyy-MM-dd") + "\n";
    m_data += "%%DocumentData: Clean7Bit\n";
    m_data += "%%BoundingBox: 0 0 " + QString::number(_width) + " " + QString::number(_height) + "\n";
    m_data += "%%EndComments\n";
    m_data += "%%BeginProlog\n";
    m_data += "save 50 dict begin /q { gsave } bind def /Q { grestore } bind def /cm { 6 array astore concat } bind def /w { setlinewidth } bind def /J { setlinecap } bind def /j { setlinejoin } bind def /M { setmiterlimit } bind def /d { setdash } bind def /m { moveto } bind def /l { lineto } bind def /c { curveto } bind def /h { closepath } bind def /re { exch dup neg 3 1 roll 5 3 roll moveto 0 rlineto 0 exch rlineto 0 rlineto closepath } bind def /S { stroke } bind def /f { fill } bind def /f* { eofill } bind def /n { newpath } bind def /W { clip } bind def /W* { eoclip } bind def /BT { } bind def /ET { } bind def /EMC { mark /EMC pdfmark } bind def /rg {setrgbcolor} bind def\n";
    m_data += "%%EndProlog\n";
    m_data += "%%BeginSetup\n";
    m_data += "%%EndSetup\n";
    m_data += "%%BeginPageSetup\n";
    m_data += "%%PageBoundingBox: 0 0 " + QString::number(_width) + " " + QString::number(_height) + "\n";

}

void EpsDocument::drawCircle(const QPointF &_position, double _radius, const QColor &_color)
{
    startPath();
    m_data += QString::number(_position.x()) + " " + QString::number(_position.y()) + " " + QString::number(_radius) + " 0 360 arc ";
    closePath();
    setColor(_color);
    fill();
}

void EpsDocument::drawSquare(const QPointF &_center, double _width, const QColor &_color)
{
    startPath();

    double x = _center.x();
    double y = _center.y();
    double w = _width / 2;

    moveTo(x-w, y-w);
    lineTo(x-w, y+w);
    lineTo(x+w, y+w);
    lineTo(x+w, y-w);
    closePath();
    setColor(_color);
    fill();
}

void EpsDocument::text(const QString &_text, double _size, double _x, double _y, int _rotation, int _horizontal)
{
    m_data += "q\n";
    m_data += "/Arial findfont\n";
    m_data += QString::number(_size) + " scalefont\n";
    m_data += "setfont\n";

    m_data += QString::number(_x) + " " + QString::number(_y) + " translate\n";
    m_data += "0 0 m\n";
    m_data += "(" + _text + ") false charpath flattenpath pathbbox\n";
    m_data += "4 2 roll pop pop\n";

    m_data += "0 0 m\n";
    m_data += QString::number(_rotation) + " rotate\n";
    alignText(_horizontal, Qt::AlignVCenter);

    m_data += "rmoveto\n";
    m_data += "(" + _text + ") show\n";
    m_data += "Q\n";
}

void EpsDocument::startPath()
{
    m_data += "n\n";
}

void EpsDocument::closePath()
{
    m_data += "h\n";
}

void EpsDocument::moveTo(const QPointF &_position)
{
    moveTo(_position.x(), _position.y());
}

void EpsDocument::moveTo(double _x, double _y)
{
    m_data += QString::number(_x) + " " + QString::number(_y) + " m\n";
}

void EpsDocument::lineTo(const QPointF &_position)
{
    lineTo(_position.x(), _position.y());
}

void EpsDocument::lineTo(double _x, double _y)
{
    m_data += QString::number(_x) + " " + QString::number(_y) + " l\n";
}

void EpsDocument::setColor(const QColor &_color)
{
    m_data += QString::number(_color.red()/255.0) + " ";
    m_data += QString::number(_color.green()/255.0) + " ";
    m_data += QString::number(_color.blue()/255.0) + " rg\n";
}

void EpsDocument::setLineWidth(double _width)
{
    m_data += QString::number(_width) + " w ";
}

void EpsDocument::setLineStyle(int _style)
{
    QString data;
    switch(_style)
    {
        case Qt::DashDotDotLine: break;
        case Qt::DashDotLine: break;
        case Qt::DashLine: data = "[4 4] 0 setdash\n"; break;
        case Qt::DotLine: data = "1 setlinecap [0 3] 0  setdash\n"; break;
        case Qt::SolidLine: data = "[] 0 setdash\n"; break;
    }
    m_data = data;
}

void EpsDocument::fill()
{
    m_data += "f\n";
}

void EpsDocument::stroke()
{
    m_data += "S\n";
}

void EpsDocument::save(const QString &_file)
{
    // add trailer
    m_data += "showpage\n";
    m_data += "%%Trailer\n";
    m_data += "end restore\n";
    m_data += "%%EOF";

    FileHandler::write(m_data.toStdString(), _file.toStdString());
}

void EpsDocument::alignText(int _horizontal, int _vertical)
{
    switch(_vertical)
    {
        case Qt::AlignVCenter: m_data += "-.5"; break;
        case Qt::AlignTop: m_data += "-1"; break;
        case Qt::AlignBottom: m_data += "0"; break;
    }
    m_data += " mul exch ";

    switch(_horizontal)
    {
        case Qt::AlignHCenter: m_data += "-.5"; break;
        case Qt::AlignLeft: m_data += "0"; break;
        case Qt::AlignRight: m_data += "-1"; break;
    }
    m_data += " mul exch\n";
}

}
