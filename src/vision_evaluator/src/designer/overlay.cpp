/// HEADER
#include "overlay.h"

/// COMPONENT
#include "connector_out.h"
#include "connector_in.h"
#include "command_meta.h"
#include "command_delete_connection.h"
#include "box_manager.h"

/// SYSTEM
#include <boost/foreach.hpp>
#include <iostream>
#include <QApplication>
#include <QTimer>
#include <QPainter>
#include <QFontMetrics>
#include <QResizeEvent>

using namespace vision_evaluator;

Overlay::Overlay(QWidget* parent)
    : QWidget(parent), next_connection_id_(0), highlight_connection_id_(-1), schema_dirty_(true)
{
    setPalette(Qt::transparent);
    setAttribute(Qt::WA_TransparentForMouseEvents);
    setFocusPolicy(Qt::NoFocus);

    repainter = new QTimer();
    repainter->setInterval(125);
    repainter->start();

    activity_marker_max_lifetime_ = 5;
    activity_marker_min_width_ = 12;
    activity_marker_max_width_ = 18;
    activity_marker_min_opacity_ = 25;
    activity_marker_max_opacity_ = 100;


    connector_radius_ = 7;

    color_in_connected = QColor(0x33, 0x33, 0xFF, 0xAA);
    color_in_disconnected = color_in_connected.darker();

    color_out_connected = QColor(0x33, 0xFF, 0x33, 0xAA);
    color_out_disconnected = color_out_connected.darker();

    setMouseTracking(true);

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
    Connection c;
    c.from = from;
    c.to = to;
    c.id = next_connection_id_;

    ++next_connection_id_;

    connections.push_back(c);

    invalidateSchema();

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
        const Connection& connection = *i;

        if(connection.from == c || connection.to == c) {
            i = connections.erase(i);
        } else {
            ++i;
        }
    }

    invalidateSchema();

    connectors_.erase(std::find(connectors_.begin(), connectors_.end(), c));

    clearActivity(c);

    repaint();
}



void Overlay::removeConnection(ConnectorOut* from, ConnectorIn* to)
{
    bool found = false;
    for(ConnectionList::iterator i = connections.begin(); i != connections.end();) {
        Connection& connection = *i;

        if(connection.from == from && connection.to == to) {
            i = connections.erase(i);
            found = true;
        } else {
            ++i;
        }
    }

    if(!found) {
        std::cerr << "could not remove connection between " << from->UUID() << " and " << to->UUID() << std::endl;
        std::cerr << "connections are:" << std::endl;
        BOOST_FOREACH(Connection& connection, connections) {
            std::cerr << " -" << connection.from->UUID() << " > " << connection.to->UUID() << std::endl;
        }
    }

    invalidateSchema();

    clearActivity(from);
    clearActivity(to);

    from->update();
    to->update();

    repaint();
}

void Overlay::drawConnection(ConnectorOut* from, ConnectorIn* to, int id)
{
    QPoint p1 = from->centerPoint();
    QPoint p2 = to->centerPoint();

    drawConnection(p1, p2, id, to->isError() || from->isError());
}


void Overlay::drawConnection(QPoint from, QPoint to, int id, bool error)
{
    double max_slack_height = 40.0;
    double mindist_for_slack = 60.0;
    double slack_smooth_distance = 300.0;

    QPoint diff = (to - from);

    double direct_length = hypot(diff.x(), diff.y());

    QPoint offset;
    QPoint delta;
    if(direct_length > mindist_for_slack) {
        double offset_factor = std::min(1.0, (direct_length - mindist_for_slack) / slack_smooth_distance);

        delta = QPoint(std::max(offset_factor * mindist_for_slack, std::abs(0.45 * (diff).x())), 0);
        offset = QPoint(0, offset_factor * max_slack_height);
    }

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

    if(highlight_connection_id_ == id) {
        painter->setPen(QPen(Qt::black, connector_radius_ * 1.75, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter->drawPath(path);

        painter->setPen(QPen(Qt::white, connector_radius_ * 1.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter->drawPath(path);
    }

    if(isConnectionWithIdSelected(id)) {
        painter->setPen(QPen(Qt::black, connector_radius_ * 1.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter->drawPath(path);

        QLinearGradient lg(from, to);
        if(error) {
            lg.setColorAt(0,Qt::darkRed);
            lg.setColorAt(1,Qt::red);
        } else {
            lg.setColorAt(0,color_out_connected.lighter(175));
            lg.setColorAt(1,color_in_connected.lighter(175));
        }

        painter->setPen(QPen(QBrush(lg), connector_radius_ * 1.3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter->drawPath(path);
    }

    Qt::PenStyle penstyle = from.x() > to.x() ? Qt::DotLine : Qt::SolidLine;
    QPen gp = QPen(QBrush(lg), connector_radius_ * 0.75, penstyle, Qt::RoundCap, Qt::RoundJoin);

    painter->setPen(gp);
    painter->drawPath(path);

    if(id < 0) {
        return;
    }

    if(schema_dirty_) {

        int rest = id;
        int r = std::min(255, rest); rest -= r;
        int g = std::min(255, rest); rest -= g;
        int b = std::min(255, rest); rest -= b;

        QRgb rgb = qRgb(r, g, b);

        QPen schema_pen = QPen(QColor(rgb), connector_radius_ * 2, Qt::SolidLine, Qt::RoundCap,Qt::RoundJoin);
        schematics_painter->setPen(schema_pen);
        schematics_painter->drawPath(path);
    }
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

        QColor color = c->isOutput() ? color_out_connected : color_in_connected;
        color.setAlpha(activity_marker_min_opacity_ + (activity_marker_max_opacity_ - activity_marker_min_opacity_) * (1-f));

        painter->setPen(QPen(color, w));
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

bool Overlay::mouseMoveEventHandler(QMouseEvent *e)
{
    QRgb rgb = schematics.pixel(e->x(), e->y());

    unsigned int id = qRed(rgb) + qGreen(rgb) + qBlue(rgb);

    if(id != 3*255) {
        highlight_connection_id_ = id;
        repaint();
    } else {
        highlight_connection_id_ = -1;
    }

    return true;
}

bool Overlay::keyPressEventHandler(QKeyEvent* e)
{
    return true;
}

bool Overlay::keyReleaseEventHandler(QKeyEvent* e)
{
    if(e->key() == Qt::Key_Delete || e->key() == Qt::Key_Backspace) {
        command::Meta::Ptr meta(new command::Meta);

        for(ConnectionList::const_iterator i = connections.begin(); i != connections.end(); ++i) {
            if(isConnectionWithIdSelected(i->id)) {
                meta->add(Command::Ptr(new command::DeleteConnection(i->from, i->to)));
            }
        }

        deselectConnections();

        BoxManager::instance().execute(meta);

        return false;
    }

    return true;
}

bool Overlay::mousePressEventHandler(QMouseEvent *e)
{
    return true;
}

bool Overlay::mouseReleaseEventHandler(QMouseEvent *e)
{
    bool shift = Qt::ShiftModifier == QApplication::keyboardModifiers();

    if(highlight_connection_id_ != -1) {
        if(e->button() == Qt::MiddleButton) {
            deleteConnectionById(highlight_connection_id_);
            return false;
        }

        if(shift) {
            if(isConnectionWithIdSelected(highlight_connection_id_)) {
                deselectConnectionById(highlight_connection_id_);
            } else {
                selectConnectionById(highlight_connection_id_, true);
            }
        } else {
            if(isConnectionWithIdSelected(highlight_connection_id_)) {
                if(noSelectedConnections() == 1) {
                    deselectConnectionById(highlight_connection_id_);
                } else {
                    selectConnectionById(highlight_connection_id_);
                }
            } else {
                selectConnectionById(highlight_connection_id_);
            }
        }
    } else if(!shift) {
        deselectConnections();

    } else {
        return true;
    }

    return false;
}

void Overlay::deleteConnectionById(int id)
{
    for(ConnectionList::const_iterator i = connections.begin(); i != connections.end(); ++i) {
        if(i->id == id) {
            BoxManager::instance().execute(Command::Ptr(new command::DeleteConnection(i->from, i->to)));
            return;
        }
    }
}

void Overlay::selectConnectionById(int id, bool add)
{
    if(!add) {
        connections_selected.clear();
    }
    connections_selected.push_back(id);

    repaint();
}

void Overlay::deselectConnectionById(int id)
{
    std::vector<int>::iterator it = std::find(connections_selected.begin(), connections_selected.end(), id);
    if(it != connections_selected.end()) {
        connections_selected.erase(it);

        repaint();
    }
}


void Overlay::deselectConnections()
{
    connections_selected.clear();

    repaint();
}

bool Overlay::isConnectionWithIdSelected(int id)
{
    return std::find(connections_selected.begin(), connections_selected.end(), id) != connections_selected.end();
}

int Overlay::noSelectedConnections()
{
    return connections_selected.size();
}

void Overlay::paintEvent(QPaintEvent* event)
{
    QPainter p(this);
    QPainter ps(&schematics);

    painter = &p;
    schematics_painter = &ps;

    if(schema_dirty_) {
        schematics_painter->fillRect(0, 0, schematics.width(), schematics.height(), Qt::white);
    }

    painter->setRenderHint(QPainter::Antialiasing);
    painter->setPen(QPen(Qt::black, 3));

    if(!temp_.empty()) {
        BOOST_FOREACH(TempConnection& temp, temp_) {

            if(dynamic_cast<ConnectorIn*> (temp.from)) {
                drawConnection(temp.to, temp.from->centerPoint(), -1);
            } else {
                drawConnection(temp.from->centerPoint(), temp.to, -1);
            }
        }
    }

    for(std::vector<std::pair<int, Connector*> >::iterator it = publisher_signals_.begin(); it != publisher_signals_.end(); ++it) {
        std::pair<int, Connector*>& p = *it;

        drawActivity(p.first, p.second);
    }

    for(ConnectionList::const_iterator i = connections.begin(); i != connections.end(); ++i) {
        const Connection& connection = *i;

        if(connection.from->isEnabled() && connection.to->isEnabled()) {
            drawConnection(connection.from, connection.to, connection.id);
        }
    }

    foreach (Connector* connector, connectors_) {
        drawConnector(connector);
    }

    //    painter->setOpacity(0.5);
    //    painter->drawImage(QPoint(0,0), schematics);

    painter = NULL;
    schematics_painter = NULL;
}

void Overlay::invalidateSchema()
{
    schema_dirty_ = true;
}

void Overlay::resizeEvent(QResizeEvent *event)
{
    schematics = QImage(event->size(), QImage::Format_RGB888);
    invalidateSchema();
}
