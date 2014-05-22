/// HEADER
#include <csapex/view/overlay.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/connection.h>
#include <csapex/model/graph.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <csapex/command/meta.h>
#include <csapex/command/delete_connection.h>
#include <csapex/command/add_fulcrum.h>
#include <csapex/command/delete_fulcrum.h>
#include <csapex/command/move_fulcrum.h>
#include <csapex/command/dispatcher.h>
#include <csapex/view/box.h>
#include <csapex/view/port.h>
#include <csapex/core/settings.h>
#include <csapex/view/widget_controller.h>

/// SYSTEM
#include <boost/foreach.hpp>
#include <iostream>
#include <cmath>
#include <QApplication>
#include <QTimer>
#include <QPainter>
#include <QFontMetrics>
#include <QResizeEvent>
#include <QMenu>

using namespace csapex;

namespace {
QRgb id2rgb(int id, int subsection)
{
    assert(id < 0xFFFF);
    assert(subsection < 0xFF);
    int raw = (id & 0xFFFF) | ((subsection & 0xFF) << 16);

    return qRgb(qRed(raw), qGreen(raw), qBlue(raw));
}

std::pair<int,int> rgb2id(QRgb rgb)
{
    int raw = (rgb & 0xFFFFFF);
    if(raw >= 0xFFFFFF) {
        return std::make_pair(-1, -1);
    }

    int id = raw & 0xFFFF;
    int subsection = (raw >> 16) & 0xFF;

    return std::make_pair(id, subsection);
}
}

Overlay::Overlay(Graph::Ptr graph, CommandDispatcher *dispatcher, WidgetControllerPtr widget_ctrl, QWidget* parent)
    : QWidget(parent), dispatcher_(dispatcher), widget_ctrl_(widget_ctrl), graph_(graph), highlight_connection_id_(-1),
      drag_connection_(-1), fulcrum_is_hovered_(false), mouse_blocked(false)
{
    setPalette(Qt::transparent);
    setAttribute(Qt::WA_TransparentForMouseEvents);
    setFocusPolicy(Qt::NoFocus);

    repainter = new QTimer();
    repainter->setInterval(125);
    repainter->start();

    activity_marker_min_width_ = 3;
    activity_marker_max_width_ = 8;
    activity_marker_min_opacity_ = 50;
    activity_marker_max_opacity_ = 90;

    connector_radius_ = 7;

    setMouseTracking(true);

    QObject::connect(repainter, SIGNAL(timeout()), this, SLOT(repaint()));
    QObject::connect(repainter, SIGNAL(timeout()), this, SLOT(tick()));

    QObject::connect(graph_.get(), SIGNAL(connectionAdded(Connection*)), this, SLOT(connectionAdded(Connection*)));
    QObject::connect(graph_.get(), SIGNAL(connectionDeleted(Connection*)), this, SLOT(connectionDeleted(Connection*)));
}



void Overlay::blockMouse(bool b)
{
    mouse_blocked = b;

    setAttribute(Qt::WA_TransparentForMouseEvents, !mouse_blocked);
}



QPen Overlay::makeSelectedLinePen(const QPoint& from, const QPoint& to)
{
    QLinearGradient lg(from, to);
    if(ccs.error) {
        lg.setColorAt(0, Qt::darkRed);
        lg.setColorAt(1, Qt::red);
    } else {
        lg.setColorAt(0, palette().foreground().color().darker());
        lg.setColorAt(1, palette().background().color().darker());
    }

    return QPen(QBrush(lg), ccs.r * 1.3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
}

QPen Overlay::makeLinePen(const QPoint& from, const QPoint& to)
{
    QLinearGradient lg(from, to);
    if(ccs.error) {
        lg.setColorAt(0,Qt::darkRed);
        lg.setColorAt(1,Qt::red);

    } else if(ccs.disabled) {
        lg.setColorAt(0,Qt::darkGray);
        lg.setColorAt(1,Qt::gray);
    } else if(ccs.async) {
        lg.setColorAt(0, QColor(0xFF, 0xCC, 0x00));
        lg.setColorAt(1, QColor(0xFF, 0xCC, 0x00));
    } else {
        QColor a = palette().foreground().color();
        QColor b = palette().background().color();
        a.setAlpha(128);
        b.setAlpha(128);
        lg.setColorAt(0, a);
        lg.setColorAt(1, b);
    }

    return QPen(QBrush(lg), ccs.r * 0.75, from.x() > to.x() ? Qt::DotLine : Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
}



void Overlay::tick()
{
}

bool Overlay::hasTempConnectionHandle() const
{
    return !drag_connection_handle_.isNull();
}

bool Overlay::keyPressEventHandler(QKeyEvent*)
{
    return true;
}

bool Overlay::keyReleaseEventHandler(QKeyEvent*)
{
//    if(e->key() == Qt::Key_Delete || e->key() == Qt::Key_Backspace) {
//        dispatcher_->execute(graph_->deleteSelectedConnectionsCmd());

//        repaint();

//        return false;
//    }

    return true;
}

bool Overlay::mousePressEventHandler(QMouseEvent *)
{
    return true;
}

bool Overlay::mouseReleaseEventHandler(QMouseEvent *e)
{
    if(splicing) {
        if(drag_connection_ == -1) {
            return false;
        }

        QPointF from = graph_->getConnectionWithId(drag_connection_)->getFulcrum(drag_sub_section_).pos;
        dispatcher_->execute(Command::Ptr(new command::MoveFulcrum(drag_connection_, drag_sub_section_, from, current_splicing_handle_)));

        splicing = false;
        splicing_requested = false;
        drag_connection_ = -1;

        fulcrum_is_hovered_ = true;

        return false;

    } else {
        if(highlight_connection_id_ != -1) {
            if(e->button() == Qt::MiddleButton) {
                if(fulcrum_is_hovered_) {
                    dispatcher_->execute(Command::Ptr(graph_->deleteConnectionFulcrumCommand(drag_connection_, drag_sub_section_)));
                } else {
                    dispatcher_->execute(graph_->deleteConnectionById(highlight_connection_id_));
                }
                return false;

            }
        }

        bool shift = Qt::ShiftModifier == QApplication::keyboardModifiers();
        widget_ctrl_->connection_selection_.handleSelection(highlight_connection_id_, shift);

        repaint();

        return true;
    }
}

bool Overlay::mouseMoveEventHandler(bool drag, QMouseEvent *e)
{
//    int x = e->x();
//    int y = e->y();

//    if(x < 0 || x >= schematics.width() || y < 0 || y >= schematics.height()) {
//        return true;
//    }

//    if(mouse_blocked) {
//        return true;
//    }


//    if(drag) {
//        if(hasTempConnectionHandle() && !splicing_requested) {
//            splicing_requested = true;
//            current_splicing_handle_ = drag_connection_handle_;

//            if(fulcrum_is_hovered_) {
//                splicing = true;

//            } else {
//                int type = Connection::Fulcrum::CURVE;
//                Command::Ptr make_fulcrum(new command::AddFulcrum(highlight_connection_id_, drag_sub_section_, drag_connection_handle_, type));
//                dispatcher_->execute(make_fulcrum);
//            }
//            return false;

//        } else if(splicing) {
//            if(current_splicing_handle_.x() != x || current_splicing_handle_.y() != y) {
//                current_splicing_handle_.setX(x);
//                current_splicing_handle_.setY(y);
//                drag_connection_handle_ = e->pos();

//                invalidateSchema();
//                repaint();
//            }
//            return false;
//        }

//    } else if(!schema_dirty_) {
//        std::pair<int, int> data = rgb2id(schematics.pixel(x,y));
//        int id = data.first;
//        int subsection = data.second;

//        if(id != -1) {
//            highlight_connection_id_ = id;
//            drag_connection_handle_ = e->pos();
//            fulcrum_is_hovered_ = false;
//            drag_sub_section_ = subsection;
//            drag_connection_ = id;

//            double closest_dist = 15;
//            Q_FOREACH(const Connection::Ptr& connection, graph_->connections_) {
//                int sub_section = 0;

//                Q_FOREACH(const Connection::Fulcrum& fulcrum, connection->getFulcrums()) {
//                    double dist = hypot(fulcrum.pos.x() - x, fulcrum.pos.y() - y);
//                    if(dist < closest_dist) {
//                        closest_dist = dist;
//                        drag_connection_handle_ = fulcrum.pos;
//                        fulcrum_is_hovered_ = true;
//                        drag_sub_section_ = sub_section;
//                        drag_connection_ = connection->id();

//                        highlight_connection_id_ = drag_connection_;
//                    }

//                    ++sub_section;
//                }
//            }

//            repaint();
//        } else {
//            highlight_connection_id_ = -1;
//            splicing = false;
//            drag_connection_handle_ = QPoint(0,0);
//        }
//    }

//    return true;
}


void Overlay::setSelectionRectangle(const QPoint &a, const QPoint &b)
{
    if(b.x() > a.x()) {
        selection_a = a;
        selection_b = b;
    } else {
        selection_a = b;
        selection_b = a;
    }
}

void Overlay::paintEvent(QPaintEvent*)
{

}

