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
    : QWidget(parent), dispatcher_(dispatcher), widget_ctrl_(widget_ctrl), graph_(graph), highlight_connection_id_(-1), schema_dirty_(true),
      drag_connection_(-1), fulcrum_is_hovered_(false), mouse_blocked(false), splicing_requested(false), splicing(false)
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


void Overlay::connectionAdded(Connection* c)
{
    QObject::connect(c, SIGNAL(fulcrum_added(Connection*)), this, SLOT(fulcrumAdded(Connection*)));
    QObject::connect(c, SIGNAL(fulcrum_deleted(Connection*)), this, SLOT(fulcrumDeleted(Connection*)));
    QObject::connect(c, SIGNAL(fulcrum_moved(Connection*)), this, SLOT(fulcrumMoved(Connection*)));
}

void Overlay::connectionDeleted(Connection*)
{

}

void Overlay::fulcrumAdded(Connection *)
{
    if(splicing_requested) {
        splicing = true;
    }
}

void Overlay::fulcrumDeleted(Connection *)
{

}

void Overlay::fulcrumMoved(Connection *)
{

}

void Overlay::blockMouse(bool b)
{
    mouse_blocked = b;

    setAttribute(Qt::WA_TransparentForMouseEvents, !mouse_blocked);
}

void Overlay::addTemporaryConnection(Connectable *from, const QPoint& end)
{
    assert(from);

    TempConnection temp;
    temp.from = from;
    temp.to = end;

    temp_.push_back(temp);
}

void Overlay::addTemporaryConnection(Connectable *from, Connectable *to)
{
    assert(from);
    assert(to);

    TempConnection temp;
    temp.from = from;
    temp.to = to->getPort()->centerPoint();

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

void Overlay::drawConnection(Connection& connection)
{
    if(!connection.from()->isOutput() || !connection.to()->isInput()) {
        return;
    }

    ConnectorOut* from = dynamic_cast<ConnectorOut*> (connection.from());
    ConnectorIn* to = dynamic_cast<ConnectorIn*> (connection.to());

    if(!from->getPort() || !to->getPort()) {
        return;
    }

    QPoint p1 = from->getPort()->centerPoint();
    QPoint p2 = to->getPort()->centerPoint();

    int id = connection.id();

    ccs = CurrentConnectionState();

    ccs.highlighted = (highlight_connection_id_ == id);
    ccs.error = (to->isError() || from->isError());
    ccs.selected = connection.isSelected();
    ccs.disabled = (!from->getPort()->isEnabled() || !to->getPort()->isEnabled());
    ccs.async = from->isAsync() || to->isAsync();
    ccs.minimized_from = from->isMinimizedSize();
    ccs.minimized_to = to->isMinimizedSize();

    drawConnection(p1, p2, id,
                   from->getPort()->isFlipped() ? Connection::Fulcrum::IN : Connection::Fulcrum::OUT,
                   to->getPort()->isFlipped() ? Connection::Fulcrum::OUT : Connection::Fulcrum::IN);

    int f = connection.activity();

    drawActivity(f, from);
    drawActivity(f, to);

    int sub_section = 0;
    Q_FOREACH(const Connection::Fulcrum& f, connection.getFulcrums()) {
        int r = 4;
        painter->setPen(QPen(Qt::black, 3));
        if(splicing && drag_connection_ == connection.id() && sub_section == drag_sub_section_) {
            painter->drawEllipse(current_splicing_handle_, r, r);
        } else {
            painter->drawEllipse(f.pos, r, r);
        }

        ++sub_section;
    }
}

void Overlay::drawConnection(const QPoint& from, const QPoint& to, int id, Connection::Fulcrum::Type from_type, Connection::Fulcrum::Type to_type)
{
    ccs.minimized = ccs.minimized_from || ccs.minimized_to;
    ccs.r = ccs.minimized ? 2 : 4;

    double max_slack_height = 40.0;
    double mindist_for_slack = 60.0;
    double slack_smooth_distance = 300.0;

    QPoint diff = (to - from);

    double direct_length = hypot(diff.x(), diff.y());

    Connection::Fulcrum current(from, from_type);
    Connection::Fulcrum last = current;

    std::vector<Connection::Fulcrum> targets;
    if(id >= 0) {
        targets = graph_->getConnectionWithId(id)->getFulcrums();
    }
    targets.push_back(Connection::Fulcrum(to, to_type));

    int sub_section = 0;

    QPoint cp1, cp2;
    QPoint tangent;

    Q_FOREACH(Connection::Fulcrum fulcrum, targets) {
        if(splicing && drag_connection_ == id && sub_section == drag_sub_section_) {
            fulcrum = Connection::Fulcrum(current_splicing_handle_, Connection::Fulcrum::HANDLE);
        }

        QPoint offset;
        QPoint delta;
        if(direct_length > mindist_for_slack) {
            double offset_factor = std::min(1.0, (direct_length - mindist_for_slack) / slack_smooth_distance);

            delta = QPoint(std::max(offset_factor * mindist_for_slack, std::abs(0.45 * (diff).x())), 0);
            offset = QPoint(0, offset_factor * max_slack_height);
        }

        QPainterPath path(current.pos);

        if(current.type == Connection::Fulcrum::OUT) {
            cp1 = current.pos + delta + offset;

        } else if(current.type == Connection::Fulcrum::IN) {
            cp1 = current.pos - delta + offset;

        } else {
            const Connection::Fulcrum& last = targets[sub_section-1];
            if(last.type == Connection::Fulcrum::LINEAR) {
                cp1 = current.pos;
            } else {
                cp1 = current.pos + tangent;
            }
        }

        if(fulcrum.type == Connection::Fulcrum::IN) {
            cp2 = fulcrum.pos - delta + offset;
        } else if(fulcrum.type == Connection::Fulcrum::OUT) {
            cp2 = fulcrum.pos + delta + offset;
        } else {
            const Connection::Fulcrum& next = targets[sub_section+1];
            tangent = (next.pos - current.pos);
            tangent*= 50.0 / hypot(diff.x(), diff.y());
            cp2 = fulcrum.pos - tangent;
        }

        if(fulcrum.type == Connection::Fulcrum::LINEAR) {
            cp2 = fulcrum.pos;
        }
        path.cubicTo(cp1, cp2, fulcrum.pos);

        if(ccs.highlighted) {
            painter->setPen(QPen(Qt::black, ccs.r + 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
            painter->drawPath(path);

            painter->setPen(QPen(Qt::white, ccs.r + 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
            painter->drawPath(path);

        } else if(ccs.selected) {
            painter->setPen(QPen(Qt::black, ccs.r + 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
            painter->drawPath(path);

            painter->setPen(makeSelectedLinePen(from, to));
            painter->drawPath(path);
        }

        painter->setPen(makeLinePen(from, to));

        painter->drawPath(path);

        if(id >= 0 && schema_dirty_) {
            QPen schema_pen = QPen(QColor(id2rgb(id, sub_section)), ccs.r * 1.75, Qt::SolidLine, Qt::RoundCap,Qt::RoundJoin);
            schematics_painter->setPen(schema_pen);
            schematics_painter->drawPath(path);
        }

        last = current;
        current = fulcrum;
        ++sub_section;
    }
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

void Overlay::drawPort(Port *p)
{
    Connectable* c = p->getAdaptee();
    bool right = c->isOutput() ^ c->getPort()->isFlipped();

    if(!c->isMinimizedSize()) {
        int font_size = 10;
        int lines = 3;

        QFont font;
        font.setPixelSize(font_size);
        painter->setFont(font);

        QString text = c->getLabel().c_str();

        if(text.length() != 0) {
            text += "\n";
        }
        text += c->getType()->name().c_str();

        QFontMetrics metrics(font);

        int dx = 160;
        int dy = lines * metrics.height();

        Port* port = c->getPort();
        QRectF rect(port->centerPoint() + QPointF(right ? 2*connector_radius_ : -2*connector_radius_-dx, -dy / 2.0), QSize(dx, dy));

        QTextOption opt(Qt::AlignVCenter | (right ? Qt::AlignLeft : Qt::AlignRight));
        QColor color = c->isOutput() ? palette().foreground().color() : palette().background().color();
        QPen p = painter->pen();
        p.setColor(color.dark());
        painter->setPen(p);
        painter->drawText(rect, text, opt);
    }
}

void Overlay::drawActivity(int life, Connectable* c)
{
    if(c->getPort()->isEnabled() && life > 0) {
        int r = std::min(Settings::activity_marker_max_lifetime_, life);
        double f = r / static_cast<double> (Settings::activity_marker_max_lifetime_);

        int min = c->getPort()->width() / 2 - 2;
        int max = min * 1.2;
        double w = min + f * (max - min);

        QColor color = c->isOutput() ? palette().foreground().color() : palette().background().color();
        color.setAlpha(activity_marker_min_opacity_ + (activity_marker_max_opacity_ - activity_marker_min_opacity_) * f);

        painter->setPen(QPen(color, w));
        painter->drawEllipse(QPointF(c->getPort()->centerPoint()), w, w);
    }
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

        QPoint from = graph_->getConnectionWithId(drag_connection_)->getFulcrum(drag_sub_section_).pos;
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
        graph_->handleConnectionSelection(highlight_connection_id_, shift);

        repaint();

        return true;
    }
}

bool Overlay::mouseMoveEventHandler(bool drag, QMouseEvent *e)
{
    int x = e->x();
    int y = e->y();

    if(x < 0 || x >= schematics.width() || y < 0 || y >= schematics.height()) {
        return true;
    }

    if(mouse_blocked) {
        return true;
    }


    if(drag) {
        if(hasTempConnectionHandle() && !splicing_requested) {
            splicing_requested = true;
            current_splicing_handle_ = drag_connection_handle_;

            if(fulcrum_is_hovered_) {
                splicing = true;

            } else {
                int type = Connection::Fulcrum::CURVE;
                Command::Ptr make_fulcrum(new command::AddFulcrum(highlight_connection_id_, drag_sub_section_, drag_connection_handle_, type));
                dispatcher_->execute(make_fulcrum);
            }
            return false;

        } else if(splicing) {
            if(current_splicing_handle_.x() != x || current_splicing_handle_.y() != y) {
                current_splicing_handle_.setX(x);
                current_splicing_handle_.setY(y);
                drag_connection_handle_ = e->pos();

                invalidateSchema();
                repaint();
            }
            return false;
        }

    } else if(!schema_dirty_) {
        std::pair<int, int> data = rgb2id(schematics.pixel(x,y));
        int id = data.first;
        int subsection = data.second;

        if(id != -1) {
            highlight_connection_id_ = id;
            drag_connection_handle_ = e->pos();
            fulcrum_is_hovered_ = false;
            drag_sub_section_ = subsection;
            drag_connection_ = id;

            double closest_dist = 15;
            Q_FOREACH(const Connection::Ptr& connection, graph_->visible_connections) {
                int sub_section = 0;

                Q_FOREACH(const Connection::Fulcrum& fulcrum, connection->getFulcrums()) {
                    double dist = hypot(fulcrum.pos.x() - x, fulcrum.pos.y() - y);
                    if(dist < closest_dist) {
                        closest_dist = dist;
                        drag_connection_handle_ = fulcrum.pos;
                        fulcrum_is_hovered_ = true;
                        drag_sub_section_ = sub_section;
                        drag_connection_ = connection->id();

                        highlight_connection_id_ = drag_connection_;
                    }

                    ++sub_section;
                }
            }

            repaint();
        } else {
            highlight_connection_id_ = -1;
            splicing = false;
            drag_connection_handle_ = QPoint(0,0);
        }
    }

    return true;
}

bool Overlay::showContextMenu(const QPoint &global_pos)
{
    if(fulcrum_is_hovered_) {
        return showFulcrumContextMenu(global_pos);
    } else {
        return showConnectionContextMenu(global_pos);
    }
}

bool Overlay::showConnectionContextMenu(const QPoint& global_pos)
{
    QPoint pos = mapFromGlobal(global_pos);
    std::pair<int, int> data = rgb2id(schematics.pixel(pos.x(),pos.y()));
    int id = data.first;

    if(id != -1) {
        QMenu menu;
        QAction* reset = new QAction("reset connection", &menu);
        menu.addAction(reset);
        QAction* del = new QAction("delete connection", &menu);
        menu.addAction(del);

        QAction* selectedItem = menu.exec(global_pos);

        if(selectedItem == del) {
            dispatcher_->execute(graph_->deleteConnectionById(id));

        } else if(selectedItem == reset) {
            dispatcher_->execute(graph_->deleteAllConnectionFulcrumsCommand(id));
        }

        return true;
    }

    return false;
}

bool Overlay::showFulcrumContextMenu(const QPoint& global_pos)
{
    Connection::Fulcrum fulc = graph_->getConnectionWithId(drag_connection_)->getFulcrum(drag_sub_section_);

    QMenu menu;
    QAction* del = new QAction("delete fulcrum", &menu);
    menu.addAction(del);

    QMenu type("change type");

    QAction* curve = new QAction("curve", &menu);
    curve->setCheckable(true);
    if(fulc.type == Connection::Fulcrum::CURVE) {
        curve->setDisabled(true);
        curve->setChecked(true);
    }
    type.addAction(curve);

    QAction* linear = new QAction("linear", &menu);
    linear->setCheckable(true);
    if(fulc.type == Connection::Fulcrum::LINEAR) {
        linear->setDisabled(true);
        linear->setChecked(true);
    }
    type.addAction(linear);

    menu.addMenu(&type);

    QAction* selectedItem = menu.exec(global_pos);

    if(selectedItem == del) {
        dispatcher_->execute(Command::Ptr(new command::DeleteFulcrum(drag_connection_, drag_sub_section_)));

    } else if(selectedItem == curve || selectedItem == linear) {
        int type = 0;
        if(selectedItem == curve) {
            type = Connection::Fulcrum::CURVE;

        } else if(selectedItem == linear) {
            type = Connection::Fulcrum::LINEAR;

        } else {
            return true;
        }

        command::Meta::Ptr cmd(new command::Meta("Change Fulcrum Type"));
        cmd->add(Command::Ptr(new command::DeleteFulcrum(drag_connection_, drag_sub_section_)));
        cmd->add(Command::Ptr(new command::AddFulcrum(drag_connection_, drag_sub_section_, fulc.pos, type)));
        dispatcher_->execute(cmd);

    } else {
        fulcrum_is_hovered_ = false;
        drag_connection_handle_ = QPoint();
        highlight_connection_id_ = -1;
    }

    return true;
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

            ccs = CurrentConnectionState();
            ccs.selected = true;

            bool flipped = temp.from->getPort()->isFlipped();

            if(temp.from->isInput()) {
                drawConnection(temp.to, temp.from->getPort()->centerPoint(), -1,
                               flipped ? Connection::Fulcrum::IN : Connection::Fulcrum::OUT,
                               Connection::Fulcrum::HANDLE);
            } else {
                drawConnection(temp.from->getPort()->centerPoint(), temp.to, -1,
                               flipped ? Connection::Fulcrum::IN : Connection::Fulcrum::OUT,
                               Connection::Fulcrum::HANDLE);
            }
        }
    }

    Q_FOREACH(Connection::Ptr connection, graph_->visible_connections) {
        drawConnection(*connection);
    }

    foreach (Node::Ptr node, graph_->nodes_) {

        Box* box = widget_ctrl_->getBox(node->getUUID());
        if(!box) {
            continue;
        }

        if(node->isError()) {
            QRectF rect(box->pos() + QPoint(0, box->height() + 8), QSize(box->width(), 64));

            QFont font;
            font.setPixelSize(8);
            painter->setFont(font);
            painter->setPen(node->errorLevel() == Node::EL_ERROR ? Qt::red : QColor(0xCC,0x99,0x00));

            QTextOption opt(Qt::AlignTop | Qt::AlignHCenter);
            painter->drawText(rect, node->errorMessage().c_str(), opt);
        }

        for(int id = 0; id < node->countInputs(); ++id) {
            Port* p = node->getInput(id)->getPort();
            if(p) {
                drawPort(p);
            }
        }
        for(int id = 0; id < node->countOutputs(); ++id) {
            ConnectorOut* o = node->getOutput(id);
            assert(o->guard_ == 0xDEADBEEF);
            Port* p = o->getPort();
            assert(p->guard_ == 0xDEADBEEF);
            if(p) {
                drawPort(p);
            }
        }
    }

    if(!drag_connection_handle_.isNull()) {
        int r = 4;
        painter->setPen(QPen(Qt::black, 1));
        painter->drawEllipse(drag_connection_handle_, r, r);
    }

    painter->setOpacity(0.35);

    if(!selection_a.isNull() && !selection_b.isNull()) {
        painter->setPen(QPen(Qt::black, 1));
        painter->setBrush(QBrush(Qt::white));
        painter->drawRect(QRect(selection_a, selection_b));
    }

    //    painter->drawImage(QPoint(0,0), schematics);

    schema_dirty_ = false;

    painter = NULL;
    schematics_painter = NULL;
}

void Overlay::invalidateSchema()
{
    schema_dirty_ = true;
}

void Overlay::refresh()
{
    invalidateSchema();
}

void Overlay::resizeEvent(QResizeEvent *event)
{
    schematics = QImage(event->size(), QImage::Format_RGB888);
    invalidateSchema();
}
