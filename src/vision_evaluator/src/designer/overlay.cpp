/// HEADER
#include "overlay.h"

/// COMPONENT
#include "connector.h"

/// SYSTEM
#include <boost/foreach.hpp>
#include <iostream>
#include <QTimer>

using namespace vision_evaluator;

Overlay::Overlay(QWidget* parent)
    : QWidget(parent)
{
    setPalette(Qt::transparent);
    setAttribute(Qt::WA_TransparentForMouseEvents);

    repainter = new QTimer();
    repainter->setInterval(125);
    repainter->start();

    QObject::connect(repainter, SIGNAL(timeout()), this, SLOT(repaint()));
    QObject::connect(repainter, SIGNAL(timeout()), this, SLOT(tick()));
}

void Overlay::drawTemporaryLine(const QLine& line)
{
    temp_line = line;
    repaint();
}

void Overlay::deleteTemporaryLine()
{
    temp_line = QLine();
    repaint();
}

void Overlay::addConnection(Connector* from, Connector* to)
{
    connections.push_back(std::make_pair(from, to));

    connect(from, SIGNAL(destroyed(QObject*)), this, SLOT(connectorRemoved(QObject*)));
    connect(to, SIGNAL(destroyed(QObject*)), this, SLOT(connectorRemoved(QObject*)));

    connect(from, SIGNAL(disconnected(QObject*)), this, SLOT(connectorRemoved(QObject*)));
    connect(to, SIGNAL(disconnected(QObject*)), this, SLOT(connectorRemoved(QObject*)));

    repaint();
}

void Overlay::connectorRemoved(QObject* o)
{
    connectorRemoved((Connector*) o);
}

void Overlay::showPublisherSignal(QPoint p)
{
    publisher_signals_.push_back(std::make_pair(10, p));
}

void Overlay::connectorRemoved(Connector* c)
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

void Overlay::removeConnection(Connector* from, Connector* to)
{
    for(ConnectionList::iterator i = connections.begin(); i != connections.end();) {
        ConnectionPair &connection = *i;

        if(connection.first == from && connection.second == to) {
            i = connections.erase(i);
        } else {
            ++i;
        }
    }

    from->update();
    to->update();

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

void Overlay::tick()
{
    for(std::vector<std::pair<int, QPoint> >::iterator it = publisher_signals_.begin(); it != publisher_signals_.end();) {
        std::pair<int, QPoint>& p = *it;

        --p.first;

        if(p.first <= 0) {
            it = publisher_signals_.erase(it);
        } else {
            ++it;
        }
    }
}

void Overlay::paintEvent(QPaintEvent* event)
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

    for(std::vector<std::pair<int, QPoint> >::iterator it = publisher_signals_.begin(); it != publisher_signals_.end(); ++it) {
        std::pair<int, QPoint>& p = *it;

        if(p.first > 0) {
            int max_radius = 20;
            int max_width = 3;
            int min_opacity = 55;

            int r = std::max(0, max_radius - p.first);
            double f = (1- r / (double) max_radius);
            int w = f * max_width;
            painter->setPen(QPen(QColor(0x33, 0x33, 0xCC, min_opacity + (255 - min_opacity) * f), w));
            painter->drawEllipse(p.second, r, r);
        }
    }

    painter = NULL;
}
