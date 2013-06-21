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

    activity_marker_max_lifetime_ = 5;
    activity_marker_min_width_ = 12;
    activity_marker_max_width_ = 18;
    activity_marker_min_opacity_ = 25;
    activity_marker_max_opacity_ = 100;

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

void Overlay::showPublisherSignal(Connector* c)
{
    publisher_signals_.push_back(std::make_pair(activity_marker_max_lifetime_, c));
}

void Overlay::clear()
{
    publisher_signals_.clear();
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
        ConnectionPair& connection = *i;

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
    QPoint offset(0, 20);

    QPoint diff = (p2 - p1);
    QPoint delta = QPoint(std::max(40, std::abs((0.45 * diff).x())), 0);
    QPoint cp1 = p1 + delta + offset;
    QPoint cp2 = p2 - delta + offset;

    QPainterPath path;
    path.moveTo(p1);
    path.cubicTo(cp1, cp2, p2);

    painter->setPen(QPen(QColor(0x33, 0x66, 0xFF, 0xDD), 3));
    painter->drawPath(path);
}

void Overlay::drawActivity(int life, Connector* c)
{
    if(life > 0) {
        int r = std::max(0, activity_marker_max_lifetime_ - life);
        double f = r / (double) activity_marker_max_lifetime_;
        int w = activity_marker_min_width_ + f * (activity_marker_max_width_ - activity_marker_min_width_);
        painter->setPen(QPen(QColor(0x33, 0x33, 0xCC, activity_marker_min_opacity_ + (activity_marker_max_opacity_ - activity_marker_min_opacity_) * (1-f)), w));
        painter->drawEllipse(c->centerPoint(), r, r);
    }
}

void Overlay::tick()
{
    for(std::vector<std::pair<int, Connector*> >::iterator it = publisher_signals_.begin(); it != publisher_signals_.end();) {
        std::pair<int, Connector*>& p = *it;

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
    for(std::vector<std::pair<int, Connector*> >::iterator it = publisher_signals_.begin(); it != publisher_signals_.end(); ++it) {
        std::pair<int, Connector*>& p = *it;

        drawActivity(p.first, p.second);
    }

    for(ConnectionList::const_iterator i = connections.begin(); i != connections.end(); ++i) {
        const ConnectionPair& connection = *i;

        drawConnection(connection.first->centerPoint(), connection.second->centerPoint());
    }


    painter = NULL;
}
