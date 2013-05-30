/// HEADER
#include "overlay.h"

/// COMPONENT
#include "connector.h"

/// SYSTEM
#include <iostream>

Overlay::Overlay(QWidget *parent)
    : QWidget(parent)
{
    setPalette(Qt::transparent);
    setAttribute(Qt::WA_TransparentForMouseEvents);
}

void Overlay::drawTemporaryLine(const QLine &line)
{
    temp_line = line;
    repaint();
}

void Overlay::deleteTemporaryLine()
{
    temp_line = QLine();
    repaint();
}

void Overlay::addConnection(Connector *from, Connector *to)
{
    connections.push_back(std::make_pair(from, to));

    connect(from, SIGNAL(destroyed(QObject*)), this, SLOT(connectorRemoved(QObject*)));
    connect(to, SIGNAL(destroyed(QObject*)), this, SLOT(connectorRemoved(QObject*)));
}

void Overlay::connectorRemoved(QObject *o)
{
    connectorRemoved((Connector*) o);
}

void Overlay::connectorRemoved(Connector *c)
{
    for(ConnectionList::iterator i = connections.begin(); i != connections.end();) {
        const ConnectionPair& connection = *i;

        if(connection.first == c || connection.second == c) {
            i = connections.erase(i);
        } else {
            ++i;
        }
    }

    repaint();
}

void Overlay::drawConnection(const QPoint& p1, const QPoint& p2)
{
    QPoint diff = (p2 - p1);
    QPoint delta = QPoint(std::max(40, std::abs((0.45 * diff).x())), 0);
    QPoint cp1 = p1 + delta;
    QPoint cp2 = p2 - delta;

    QPainterPath path;
    path.moveTo(p1);
    path.cubicTo(cp1, cp2, p2);

    painter->drawPath(path);
}

void Overlay::paintEvent(QPaintEvent *event)
{
    QPainter p(this);

    painter = &p;

    painter->setRenderHint(QPainter::Antialiasing);
    painter->setPen(QPen(Qt::black, 3));

    if(!temp_line.isNull()) {
        drawConnection(temp_line.p1(), temp_line.p2());
    }

    for(ConnectionList::const_iterator i = connections.begin(); i != connections.end(); ++i) {
        const ConnectionPair& connection = *i;

        painter->setPen(QPen(Qt::red, 2));
        drawConnection(connection.first->centerPoint(), connection.second->centerPoint());
    }

    painter = NULL;
}
