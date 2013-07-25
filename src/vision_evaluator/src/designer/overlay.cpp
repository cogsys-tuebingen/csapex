/// HEADER
#include "overlay.h"

/// COMPONENT
#include "connector_out.h"
#include "connector_in.h"

/// SYSTEM
#include <boost/foreach.hpp>
#include <iostream>
#include <QTimer>
#include <QPainter>
#include <QFontMetrics>

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


    connector_radius_ = 7;

    color_connection = QColor(0xCC, 0xDD, 0xEE, 0xFF);

    color_in_connected = QColor(0x33, 0x33, 0xFF, 0xFF);
    color_in_disconnected = QColor(0xDD, 0xEE, 0xFF, 0xFF);

    color_out_connected = QColor(0x77, 0x33, 0xFF, 0xFF);
    color_out_disconnected = QColor(0xFF, 0xEE, 0xFF, 0xFF);

    QObject::connect(repainter, SIGNAL(timeout()), this, SLOT(repaint()));
    QObject::connect(repainter, SIGNAL(timeout()), this, SLOT(tick()));
}

void Overlay::addTemporaryConnection(Connector *from, const QPoint& end)
{
    TempConnection temp;
    temp.from = from;
    temp.to = end;

    temp_.push_back(temp);
}

void Overlay::addTemporaryConnection(Connector *from, Connector *to)
{
    TempConnection temp;
    temp.from = from;
    temp.to = to->centerPoint();

    temp_.push_back(temp);
}

void Overlay::deleteTemporaryConnections()
{
    temp_.clear();
}

void Overlay::deleteTemporaryConnectionsAndRepaint()
{
    deleteTemporaryConnections();
    repaint();
}

void Overlay::addConnection(ConnectorOut* from, ConnectorIn* to)
{
    connections.push_back(std::make_pair(from, to));

    repaint();
}

void Overlay::connectorRemoved(QObject* o)
{
    connectorRemoved((Connector*) o);
}

void Overlay::connectorAdded(QObject* o)
{
    connectorAdded((Connector*) o);
}

void Overlay::connectorEnabled(Connector *c)
{
//    connectorAdded(c);
}

void Overlay::connectorDisabled(Connector* c)
{
//    connectorRemoved(c);
}

void Overlay::showPublisherSignal(ConnectorIn *c)
{
    showPublisherSignal((Connector*) c);
}

void Overlay::showPublisherSignal(ConnectorOut *c)
{
    showPublisherSignal((Connector*) c);
}

void Overlay::showPublisherSignal(Connector* c)
{
    publisher_signals_.push_back(std::make_pair(activity_marker_max_lifetime_, c));
}

void Overlay::clear()
{
    publisher_signals_.clear();
}


void Overlay::connectorAdded(Connector *c)
{
    connectors_.push_back(c);

    connect(c, SIGNAL(destroyed(QObject*)), this, SLOT(connectorRemoved(QObject*)));
    connect(c, SIGNAL(disconnected(QObject*)), this, SLOT(connectorRemoved(QObject*)));
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

    connectors_.erase(std::find(connectors_.begin(), connectors_.end(), c));

    clearActivity(c);

    repaint();
}



void Overlay::removeConnection(ConnectorOut* from, ConnectorIn* to)
{
    bool found = false;
    for(ConnectionList::iterator i = connections.begin(); i != connections.end();) {
        ConnectionPair& connection = *i;

        if(connection.first == from && connection.second == to) {
            i = connections.erase(i);
            found = true;
        } else {
            ++i;
        }
    }

    if(!found) {
        std::cerr << "could not remove connection between " << from->UUID() << " and " << to->UUID() << std::endl;
        std::cerr << "connections are:" << std::endl;
        BOOST_FOREACH(ConnectionPair& connection, connections) {
            std::cerr << " -" << connection.first->UUID() << " > " << connection.second->UUID() << std::endl;
        }
    }

    clearActivity(from);
    clearActivity(to);

    from->update();
    to->update();

    repaint();
}

void Overlay::drawConnection(ConnectorOut* from, ConnectorIn* to)
{
    QPoint p1 = from->centerPoint();
    QPoint p2 = to->centerPoint();

    drawConnection(p1, p2, to->isError() || from->isError());
}


void Overlay::drawConnection(QPoint from, QPoint to, bool error)
{
    QPoint offset(0, 20);

    QPoint diff = (to - from);
    QPoint delta = QPoint(std::max(40, std::abs((0.45 * diff).x())), 0);
    QPoint cp1 = from + delta + offset;
    QPoint cp2 = to - delta + offset;

    QPainterPath path;
    path.moveTo(from);
    path.cubicTo(cp1, cp2, to);

    QLinearGradient lg(from, to);
    if(error) {
        lg.setColorAt(0,Qt::darkRed);
        lg.setColorAt(1,Qt::red);
    } else {
        lg.setColorAt(0,color_out_connected.lighter());
        lg.setColorAt(1,color_in_connected.lighter());
    }
    QPen gp = QPen(Qt::black, connector_radius_ * 0.75, Qt::DotLine, Qt::RoundCap,Qt::RoundJoin);
    gp.setBrush(QBrush(lg));

    painter->setPen(gp);
    painter->drawPath(path);
}

void Overlay::drawConnector(Connector *c)
{
    bool output = c->isOutput();

    QColor color;

    if(c->isError()) {
        color= Qt::red;
    } else if(!c->isEnabled()) {
        color= Qt::gray;
    } else {
        if(c->isConnected()){
            color = output ? color_out_connected : color_in_connected;
        } else {
            color = output ? color_out_disconnected : color_in_disconnected;
        }
    }

    painter->setBrush(QBrush(color, Qt::SolidPattern));
    painter->setPen(QPen(color.darker(), 2));

    int font_size = 10;
    int lines = 3;
    painter->drawEllipse(c->centerPoint(), connector_radius_, connector_radius_);

    QTextOption opt(Qt::AlignVCenter | (output ? Qt::AlignLeft : Qt::AlignRight));

    QFont font;
    font.setPixelSize(font_size);
    painter->setFont(font);

    QString text = c->getLabel().c_str();
    if(text.length() != 0) {
        text += "\n";
    }
    text += c->getType()->name().c_str();

    QFontMetrics metrics(font);

    int dx = 80;
    int dy = lines * metrics.height();

    QRectF rect(c->centerPoint() + QPointF(output ? 2*connector_radius_ : -2*connector_radius_-dx, -dy / 2.0), QSize(dx, dy));

    painter->drawText(rect, text, opt);
}

void Overlay::drawActivity(int life, Connector* c)
{
    if(c->isEnabled() && life > 0) {
        int r = std::max(0, activity_marker_max_lifetime_ - life);
        double f = r / (double) activity_marker_max_lifetime_;
        int w = activity_marker_min_width_ + f * (activity_marker_max_width_ - activity_marker_min_width_);
        painter->setPen(QPen(QColor(0x33, 0x33, 0xCC, activity_marker_min_opacity_ + (activity_marker_max_opacity_ - activity_marker_min_opacity_) * (1-f)), w));
        painter->drawEllipse(c->centerPoint(), r, r);
    }
}

void Overlay::clearActivity(Connector *c)
{
    for(std::vector<std::pair<int, Connector*> >::iterator it = publisher_signals_.begin(); it != publisher_signals_.end();) {
        std::pair<int, Connector*>& p = *it;

        if(p.second == c) {
            it = publisher_signals_.erase(it);
        } else {
            ++it;
        }
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

    if(!temp_.empty()) {
        BOOST_FOREACH(TempConnection& temp, temp_) {

            if(dynamic_cast<ConnectorIn*> (temp.from)) {
                drawConnection(temp.to, temp.from->centerPoint());
            } else {
                drawConnection(temp.from->centerPoint(), temp.to);
            }
        }
    }
    for(std::vector<std::pair<int, Connector*> >::iterator it = publisher_signals_.begin(); it != publisher_signals_.end(); ++it) {
        std::pair<int, Connector*>& p = *it;

        drawActivity(p.first, p.second);
    }

    for(ConnectionList::const_iterator i = connections.begin(); i != connections.end(); ++i) {
        const ConnectionPair& connection = *i;

        if(connection.first->isEnabled() && connection.second->isEnabled()) {
            drawConnection(connection.first, connection.second);
        }
    }

    foreach (Connector* connector, connectors_) {
        drawConnector(connector);
    }

    painter = NULL;
}
