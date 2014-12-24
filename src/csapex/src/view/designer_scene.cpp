/// HEADER
#include <csapex/view/designer_scene.h>

/// COMPONENT
#include <csapex/model/connectable.h>
#include <csapex/model/graph.h>
#include <csapex/view/widget_controller.h>
#include <csapex/view/port.h>
#include <csapex/model/node.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/connection.h>
#include <csapex/view/box.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/slot.h>
#include <csapex/signal/trigger.h>
#include <csapex/core/settings.h>
#include <csapex/command/dispatcher.h>
#include <csapex/command/add_fulcrum.h>
#include <csapex/command/move_fulcrum.h>
#include <csapex/command/modify_fulcrum.h>
#include <csapex/view/fulcrum_widget.h>
#include <csapex/utility/movable_graphics_proxy_widget.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <QtGui>
#include <QtOpenGL>
#include <GL/glu.h>

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

using namespace csapex;

const float DesignerScene::ARROW_LENGTH = 10.f;

namespace {
QRgb id2rgb(int id, int subsection)
{
    apex_assert_hard(id < 0xFFFF);
    apex_assert_hard(subsection < 0xFF);
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

QWidget* topLevelParentWidget (QWidget* widget)
{
    while (widget -> parentWidget()) widget = widget -> parentWidget() ;
    return widget ;
}

QPointF centerPoint(Port* port)
{

    QPointF pos;
    QWidget* widget = port;
    while (widget -> parentWidget()) {
        widget = widget -> parentWidget();
        pos += widget->pos();
    }

    if(!port->isVisible()) {
        QSizeF s = widget->geometry().size() * 0.5;
        return widget->pos() + QPointF(s.width(), s.height());
    } else {
        return pos + port->centerPoint();
    }
}
}



DesignerScene::DesignerScene(GraphPtr graph, CommandDispatcher *dispatcher, WidgetControllerPtr widget_ctrl)
    : graph_(graph), dispatcher_(dispatcher), widget_ctrl_(widget_ctrl),
      draw_grid_(false), draw_schema_(false), scale_(1.0), overlay_threshold_(0.45),
      highlight_connection_id_(-1), highlight_connection_sub_id_(-1), schema_dirty_(false)
{
    background_ = QPixmap::fromImage(QImage(":/background.png"));

    activity_marker_min_width_ = 3;
    activity_marker_max_width_ = 8;
    activity_marker_min_opacity_ = 50;
    activity_marker_max_opacity_ = 90;

    connector_radius_ = 7;

    output_color_ = QColor(0xFF, 0xCC, 0x00);
    input_color_ = QColor(0xFF, 0x00, 0xCC);

    QObject::connect(graph_.get(), SIGNAL(connectionAdded(Connection*)), this, SLOT(connectionAdded(Connection*)), Qt::QueuedConnection);
    QObject::connect(graph_.get(), SIGNAL(connectionDeleted(Connection*)), this, SLOT(connectionDeleted(Connection*)), Qt::QueuedConnection);
}

DesignerScene::~DesignerScene()
{

}

void DesignerScene::enableGrid(bool draw)
{
    if(draw != draw_grid_) {
        draw_grid_ = draw;

        update();
    }
}

void DesignerScene::enableSchema(bool draw)
{
    if(draw != draw_schema_) {
        draw_schema_ = draw;
        update();
    }
}

void DesignerScene::setScale(double scale)
{
    scale_ = scale;
    invalidateSchema();
}

void DesignerScene::drawBackground(QPainter *painter, const QRectF &rect)
{
    QGraphicsScene::drawBackground(painter, rect);

    if(isEmpty()) {
        QFile file(QString::fromStdString(Settings::defaultConfigPath() + "cfg/intro.html"));

        if(file.exists() && file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            painter->resetTransform();

            QTextDocument doc(this);
            doc.setUndoRedoEnabled(false);
            doc.setHtml(file.readAll());
            doc.setUseDesignMetrics(true);
            doc.drawContents(painter);
        }

    } else if(draw_grid_) {
        bool spt = painter->renderHints() & QPainter::SmoothPixmapTransform;
        painter->setRenderHint(QPainter::SmoothPixmapTransform, scale_ < 1.0);

        qreal o = painter->opacity();
        static const double max_distance = 0.4;
        if(scale_ < 1.0) {
            qreal op = (scale_ - max_distance) / max_distance;
            painter->setOpacity(op);
        }

        painter->drawTiledPixmap(sceneRect(), background_);

        painter->setOpacity(o);

        painter->setRenderHint(QPainter::SmoothPixmapTransform, spt);
    }
}

void DesignerScene::drawForeground(QPainter *painter, const QRectF &rect)
{
    QGraphicsScene::drawForeground(painter, rect);

    if(isEmpty()) {
        return;
    }

    // check if we need to update the schematics
    QSize scene_size(sceneRect().width(), sceneRect().height());
    if(schematics.isNull() || schematics.size() != scene_size ) {
        schematics = QImage(scene_size, QImage::Format_RGB888);
        schema_dirty_ = true;
    }

    // make schematics renderer
    QPainter ps(&schematics);
    ps.setWindow(sceneRect().toRect());
    schematics_painter = &ps;

    if(schema_dirty_) {
        schematics_painter->fillRect(sceneRect(), -1);
    }

    // set drawing params
    painter->setRenderHint(QPainter::Antialiasing);
    painter->setPen(QPen(Qt::black, 3));

    // check if we have temporary connections
    if(!temp_.empty()) {
        foreach(const TempConnection& temp, temp_) {
            if(temp.is_connected) {
                drawConnection(painter, Connection(temp.from, temp.to_c, -1));

            } else {
                ccs = CurrentConnectionState();
                Port* fromp = widget_ctrl_->getPort(temp.from);

                ccs.start_pos = UNDEFINED;
                ccs.end_pos = UNDEFINED;


                if(temp.from->isInput()) {
                    drawConnection(painter, temp.to_p, centerPoint(fromp), -1);
                } else {
                    drawConnection(painter, centerPoint(fromp), temp.to_p, -1);
                }
            }
        }
    }

    // draw all connections
    Q_FOREACH(Connection::Ptr connection, graph_->connections_) {
        drawConnection(painter, *connection);
    }

    // augment nodes
    foreach(NodeWorker* node_worker, graph_->getAllNodeWorkers()) {
        Node* node = node_worker->getNode();

        NodeBox* box = widget_ctrl_->getBox(node_worker->getUUID());
        if(!box) {
            continue;
        }

        // draw error message
        if(node->isError()) {
            QRectF rect(box->pos() + QPoint(0, box->height() + 8), QSize(box->width(), 64));

            QFont font;
            font.setPixelSize(8);
            painter->setFont(font);
            painter->setPen(node->errorLevel() == Node::EL_ERROR ? Qt::red : QColor(0xCC,0x99,0x00));

            QTextOption opt(Qt::AlignTop | Qt::AlignHCenter);
            painter->drawText(rect, node->errorMessage().c_str(), opt);
        }

        // draw box overlay
        if(scale_ < overlay_threshold_) {
            double o = 1.0;
            double thresh = overlay_threshold_ * 0.75;
            if(scale_ > thresh) {
                o = (overlay_threshold_ - scale_) / (overlay_threshold_ - thresh);
            }
            painter->setOpacity(o * 0.75);

            QBrush brush(QColor(0xCC, 0xCC, 0xCC));
            QPen pen(brush, 1.0);
            painter->setPen(pen);
            painter->setBrush(brush);
            painter->drawRect(box->geometry());

            painter->setRenderHint(QPainter::Antialiasing);
            painter->setPen(QPen(QColor(0,0,0)));
            QFont font = painter->font();
            font.setPixelSize(10 / scale_);
            font.setBold(true);
            painter->setFont(font);
            QTextOption opt(Qt::AlignCenter);
            opt.setWrapMode(QTextOption::WrapAnywhere);

            std::string s = box->getLabel();
            int grow = 10 / scale_;
            painter->drawText(box->geometry().adjusted(-grow, -grow, grow, grow), s.c_str(), opt);
        }

        // draw port information (in)
        Q_FOREACH(Input* input, node_worker->getMessageInputs()) {
            Port* p = widget_ctrl_->getPort(input);
            if(p) {
                drawPort(painter, box, p);
            }
        }
        // draw port information (out)
        Q_FOREACH(Output* output, node_worker->getMessageOutputs()) {
            Port* p = widget_ctrl_->getPort(output);
            if(p) {
                drawPort(painter, box, p);
            }
        }

        // draw slots
        Q_FOREACH(Slot* slot, node_worker->getSlots()) {
            Port* p = widget_ctrl_->getPort(slot);
            if(p) {
                drawPort(painter, box, p);
            }
        }
        // draw triggers
        Q_FOREACH(Trigger* trigger, node_worker->getTriggers()) {
            Port* p = widget_ctrl_->getPort(trigger);
            if(p) {
                drawPort(painter, box, p);
            }
        }
    }

    if(draw_schema_){
        painter->setOpacity(0.35);
        painter->drawImage(sceneRect().topLeft(), schematics);
    }

    schema_dirty_ = false;

    schematics_painter = NULL;
}

void DesignerScene::drawItems(QPainter *painter, int numItems,
                              QGraphicsItem *items[],
                              const QStyleOptionGraphicsItem options[],
                              QWidget *widget)
{
    QGraphicsScene::drawItems(painter, numItems, items, options, widget);
}

void DesignerScene::mousePressEvent(QGraphicsSceneMouseEvent *e)
{
    /*if(e->button() == Qt::RightButton && highlight_connection_id_ >= 0 && items(e->scenePos(), Qt::ContainsItemShape, Qt::AscendingOrder).empty()) {
        e->accept();
        showConnectionContextMenu();
        return;

    } else*/ if(e->button() == Qt::MiddleButton && highlight_connection_id_ >= 0) {
        dispatcher_->execute(graph_->deleteConnectionById(highlight_connection_id_));
        return;
    }

    QGraphicsScene::mousePressEvent(e);

    if(!e->isAccepted() && e->button() == Qt::LeftButton) {
        if(highlight_connection_id_ >= 0) {
            QPoint pos = e->scenePos().toPoint();
            dispatcher_->execute(Command::Ptr(new command::AddFulcrum(highlight_connection_id_, highlight_connection_sub_id_, pos, 0)));
            e->accept();

            // allow moving the fulcrum directly
            QGraphicsScene::mousePressEvent(e);
        }
    }
}

void DesignerScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *e)
{
    QGraphicsScene::mouseReleaseEvent(e);
}

void DesignerScene::mouseMoveEvent(QGraphicsSceneMouseEvent *e)
{
    QGraphicsScene::mouseMoveEvent(e);

    QPoint pos = (e->scenePos() - sceneRect().topLeft()).toPoint();
    if(!schematics.rect().contains(pos)) {
        return;
    }

    std::pair<int, int> data = rgb2id(schematics.pixel(pos.x(),pos.y()));

    if(data.first != highlight_connection_id_) {
        highlight_connection_id_ = data.first;
        highlight_connection_sub_id_ = data.second;
        update();
    }
}

int DesignerScene::getHighlightedConnectionId() const
{
    return highlight_connection_id_;
}

bool DesignerScene::isEmpty() const
{
    return graph_->nodes_.empty();
}

void DesignerScene::connectionAdded(Connection* c)
{
    foreach(FulcrumPtr f, c->getFulcrums()) {
        fulcrumAdded(f.get());
    }

    QObject::connect(c, SIGNAL(fulcrum_added(Fulcrum *)), this, SLOT(fulcrumAdded(Fulcrum *)));
    QObject::connect(c, SIGNAL(fulcrum_deleted(Fulcrum*)), this, SLOT(fulcrumDeleted(Fulcrum*)), Qt::DirectConnection);
    QObject::connect(c, SIGNAL(fulcrum_moved(Fulcrum*,bool)), this, SLOT(fulcrumMoved(Fulcrum *, bool)));
    QObject::connect(c, SIGNAL(fulcrum_moved_handle(Fulcrum*,bool,int)), this, SLOT(fulcrumHandleMoved(Fulcrum *, bool, int)));
    QObject::connect(c, SIGNAL(fulcrum_type_changed(Fulcrum*,int)), this, SLOT(fulcrumTypeChanged(Fulcrum*,int)));

    invalidateSchema();
}

void DesignerScene::connectionDeleted(Connection*)
{
    invalidateSchema();
}

void DesignerScene::boxMoved(NodeBox *box)
{
    MovableGraphicsProxyWidget* proxy = widget_ctrl_->getProxy(box->getNodeWorker()->getUUID());
    proxy->setPos(box->pos());
    invalidateSchema();

    QObject::connect(proxy, SIGNAL(geometryChanged()), this, SLOT(invalidateSchema()));
}

void DesignerScene::fulcrumAdded(Fulcrum * f)
{
    FulcrumWidget* w = new FulcrumWidget(f, dispatcher_);
    addItem(w);
    fulcrum_2_widget_[f] = w;
    fulcrum_last_pos_[f] = f->pos();
    fulcrum_last_type_[f] = f->type();
    fulcrum_last_hin_[f] = f->handleIn();
    fulcrum_last_hout_[f] = f->handleOut();

    clearSelection();
    w->setSelected(true);
    setFocusItem(w, Qt::MouseFocusReason);

    invalidateSchema();
}

void DesignerScene::fulcrumDeleted(Fulcrum* f)
{
    std::map<Fulcrum*, FulcrumWidget*>::iterator pos = fulcrum_2_widget_.find(f);
    apex_assert_hard(pos != fulcrum_2_widget_.end());

    delete pos->second;
    fulcrum_2_widget_.erase(pos);

    invalidateSchema();
}

void DesignerScene::fulcrumMoved(Fulcrum * f, bool dropped)
{
    if(dropped) {
        dispatcher_->execute(Command::Ptr(new command::MoveFulcrum(f->connectionId(), f->id(), fulcrum_last_pos_[f], f->pos())));
        fulcrum_last_pos_[f] = f->pos();
    }
    invalidateSchema();
}

void DesignerScene::fulcrumHandleMoved(Fulcrum * f, bool dropped, int /*which*/)
{
    if(dropped) {
        dispatcher_->execute(Command::Ptr(new command::ModifyFulcrum(f->connectionId(), f->id(),
                                                                     fulcrum_last_type_[f], fulcrum_last_hin_[f], fulcrum_last_hout_[f],
                                                                     f->type(), f->handleIn(), f->handleOut())));
        fulcrum_last_type_[f] = f->type();
        fulcrum_last_hin_[f] = f->handleIn();
        fulcrum_last_hout_[f] = f->handleOut();

    }
    invalidateSchema();
}

void DesignerScene::fulcrumTypeChanged(Fulcrum */*f*/, int /*type*/)
{
    invalidateSchema();
}

void DesignerScene::addTemporaryConnection(Connectable *from, const QPointF& end)
{
    apex_assert_hard(from);

    TempConnection temp(false);
    temp.from = from;
    temp.to_p = end;

    temp_.push_back(temp);

    update();
}

void DesignerScene::previewConnection(Connectable *from, Connectable *to)
{
    deleteTemporaryConnections();
    addTemporaryConnection(from, to);
    update();
}

void DesignerScene::addTemporaryConnection(Connectable *from, Connectable *to)
{
    apex_assert_hard(from);
    apex_assert_hard(to);

    Connectable* input;
    Connectable* output;
    if(from->isInput()) {
        input = from;
        output = to;

    } else {
       input = to;
        output = from;
    }

    apex_assert_hard(input);
    apex_assert_hard(output);


    TempConnection temp(true);
    temp.from = output;
    temp.to_c = input;

    temp_.push_back(temp);
}

void DesignerScene::deleteTemporaryConnections()
{
    temp_.clear();
}

void DesignerScene::deleteTemporaryConnectionsAndRepaint()
{
    deleteTemporaryConnections();
    update();
}

void DesignerScene::drawConnection(QPainter *painter, const Connection& connection)
{
    if(!connection.from()->isOutput() || !connection.to()->isInput()) {
        return;
    }

    Connectable* from = connection.from();
    Connectable* to = connection.to();

    Port* fromp = widget_ctrl_->getPort(from);
    Port* top = widget_ctrl_->getPort(to);

    if(!fromp || !top) {
        return;
    }

    QPointF p1 = centerPoint(fromp);
    QPointF p2 = centerPoint(top);

    int id = connection.id();

    ccs = CurrentConnectionState();

    ccs.highlighted = (highlight_connection_id_ == id);
    ccs.error = (to->isError() || from->isError());
    ccs.disabled = (!from->isEnabled() || !to->isEnabled());
    ccs.blocked_from = from->isBlocked();
    ccs.blocked_to = to->isBlocked();
    ccs.minimized_from = fromp->isMinimizedSize();
    ccs.minimized_to = top->isMinimizedSize();
    ccs.hidden_from = !fromp->isVisible();
    ccs.hidden_to = !top->isVisible();

    bool is_msg = dynamic_cast<Input*>(from) || dynamic_cast<Output*>(from);

    ccs.start_pos = is_msg ? (fromp->isFlipped() ? LEFT : RIGHT) : BOTTOM;
    ccs.end_pos = is_msg ? (top->isFlipped() ? RIGHT : LEFT) : TOP;

    drawConnection(painter, p1, p2, id);
}


QPointF DesignerScene::offset(const QPointF& vector, Position position, double offset)
{
    QPointF result = vector;
    switch(position) {
    case LEFT:
        result.setX(result.x() -offset);
        break;
    case RIGHT:
        result.setX(result.x() + offset);
        break;
    case TOP:
        result.setY(result.y() -offset);
        break;
    case BOTTOM:
        result.setY(result.y() + offset);
        break;

    default:
        break;
    }
    return result;
}


void DesignerScene::drawConnection(QPainter *painter, const QPointF& from, const QPointF& real_to, int id)
{
    QPointF to = offset(real_to, ccs.end_pos, ARROW_LENGTH);

    painter->setRenderHint(QPainter::Antialiasing);

    double scale_factor = 1.0 / scale_;
    if(scale_factor < 1.0) {
        scale_factor = 1.0;
    }

    ccs.minimized = ccs.minimized_from || ccs.minimized_to;
    ccs.r = ccs.minimized ? 2 : 4;
    ccs.r *= scale_factor;

    double max_slack_height = 40.0;
    double mindist_for_slack = 60.0;
    double slack_smooth_distance = 300.0;

    QPointF diff = (to - from);

    double direct_length = hypot(diff.x(), diff.y());


    Fulcrum::Ptr first(new Fulcrum(NULL, from, Fulcrum::OUT, from, from));
    Fulcrum::Ptr current = first;
    Fulcrum::Ptr last = current;

    std::vector<Fulcrum::Ptr> targets;
    if(id >= 0) {
        Connection::Ptr connection = graph_->getConnectionWithId(id);
        targets = connection->getFulcrums();
    }
    targets.push_back(Fulcrum::Ptr(new Fulcrum(NULL, to, Fulcrum::IN, to, to)));

    int sub_section = 0;

    QPointF cp1, cp2;

    // paths
    typedef std::pair<QPainterPath, int> Path;
    std::vector<Path> paths;

    // generate lines
    Q_FOREACH(Fulcrum::Ptr next, targets) {
        QPoint y_offset;
        double x_offset = 0;
        if(direct_length > mindist_for_slack) {
            double offset_factor = std::min(1.0, (direct_length - mindist_for_slack) / slack_smooth_distance);

            x_offset = std::max(offset_factor * mindist_for_slack, std::abs(0.45 * (diff).x()));
            y_offset = QPoint(0, offset_factor * max_slack_height);
        }

        QPainterPath path(current->pos());

        if(current->type() == Fulcrum::OUT) {
            cp1 = offset(current->pos(), ccs.start_pos, x_offset) + y_offset;

        } else if(current->type() == Fulcrum::IN) {
            cp1 = offset(current->pos(), ccs.end_pos, x_offset) + y_offset;

        } else {
            const Fulcrum::Ptr& last = targets[sub_section-1];
            if(last->type() == Fulcrum::LINEAR) {
                cp1 = current->pos();
            } else {
                cp1 = current->pos() + current->handleOut();
            }
        }

        if(next->type() == Fulcrum::OUT) {
            cp2 = offset(next->pos(), ccs.start_pos, x_offset) + y_offset;

        } else if(next->type() == Fulcrum::IN) {
            cp2 = offset(next->pos(), ccs.end_pos, x_offset) + y_offset;

        } else {
            if(next->type() == Fulcrum::LINEAR) {
                cp2 = next->pos();
            } else {
                cp2 = next->pos() + next->handleIn();
            }
        }

        path.cubicTo(cp1, cp2, next->pos());

        paths.push_back(std::make_pair(path, sub_section));

        last = current;
        current = next;
        ++sub_section;
    }

    // arrow
    QPolygonF arrow;
    QPointF a = offset(QPointF(0,0), ccs.end_pos, ARROW_LENGTH) * scale_factor;

    QPointF side(a.y()/2.0, a.x()/2.0);
    arrow.append(to - a);
    arrow.append(to - side);
    arrow.append(to + side);
    arrow.append(to - a);

    QPainterPath arrow_path;
    arrow_path.addPolygon(arrow);

    // reset brush if it is set
    painter->setBrush(QBrush());

    // draw
    if(ccs.highlighted) {
        painter->setPen(QPen(Qt::black, ccs.r + 6*scale_factor, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        foreach(const Path& path, paths) {
            painter->drawPath(path.first);
        }
        painter->drawPath(arrow_path);

        painter->setPen(QPen(Qt::white, ccs.r + 3*scale_factor, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        foreach(const Path& path, paths) {
            painter->drawPath(path.first);
        }
        painter->drawPath(arrow_path);

    } else {
        QLinearGradient bg_gradient(from, to);
        bg_gradient.setColorAt(0, QColor(0,0,0, ccs.minimized_from ? 0 : 255));
        bg_gradient.setColorAt(1, QColor(0,0,0, ccs.minimized_to ? 0 : 255));

        painter->setPen(QPen(QBrush(bg_gradient), ccs.r + 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        foreach(const Path& path, paths) {
            painter->drawPath(path.first);
        }
        painter->drawPath(arrow_path);
    }

    QColor color_start = output_color_;
    QColor color_end = input_color_;

    if(ccs.blocked_from || ccs.blocked_to) {
        color_start = ccs.blocked_from ? Qt::yellow : Qt::black;
        color_end = ccs.blocked_to ? Qt::yellow : Qt::black;

    } else if(ccs.error) {
        color_start = Qt::darkRed;
        color_end = Qt::red;

    } else if(ccs.disabled) {
        color_start = Qt::darkGray;
        color_end = Qt::gray;
    }

    if(ccs.hidden_from) {
        color_start.setAlpha(60);
    }
    if(ccs.hidden_to) {
        color_end.setAlpha(60);
    }

    QLinearGradient lg(from, to);
    lg.setColorAt(0, color_start);
    lg.setColorAt(1, color_end);

    painter->setPen(QPen(QBrush(lg), ccs.r * 0.75, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

    foreach(const Path& path, paths) {
        painter->drawPath(path.first);

        if(id >= 0 && schema_dirty_) {
            QPen schema_pen = QPen(QColor(id2rgb(id, path.second)), ccs.r * 1.75, Qt::SolidLine, Qt::RoundCap,Qt::RoundJoin);
            schematics_painter->setPen(schema_pen);
            schematics_painter->drawPath(path.first);
        }
    }
    painter->setBrush(color_end);
    painter->setPen(QPen(painter->brush(), 1.0));
    painter->drawPath(arrow_path);

    if(id >= 0 && schema_dirty_) {
        schematics_painter->drawPath(arrow_path);
    }
}

void DesignerScene::drawPort(QPainter *painter, NodeBox* box, Port *p)
{
    // reset brush if it is set
    painter->setBrush(QBrush());

    Connectable* c = p->getAdaptee();
    bool is_message = (dynamic_cast<Slot*>(c) == NULL && dynamic_cast<Trigger*>(c) == NULL);

    if(!p->isMinimizedSize()) {
        int font_size = 10;
        int lines = 2;

        QFont font;
        font.setPixelSize(font_size);
        painter->setFont(font);

        QString text = c->getLabel().c_str();

        if(is_message) {
            if(text.length() != 0) {
                text += "\n";
            }
            text += c->getType()->name().c_str();
            ++lines;
        }


        QFontMetrics metrics(font);

        int dx = 160;
        int dy = lines * metrics.height();

        QPointF pos = box->pos() + p->centerPoint();
        QRectF rect;
        QTextOption opt;
        if(is_message) {
            bool right = c->isOutput() ^ p->isFlipped();
            rect = QRectF(pos + QPointF(right ? 2*connector_radius_ : -2*connector_radius_-dx, -dy / 2.0), QSize(dx, dy));
            opt = QTextOption(Qt::AlignVCenter | (right ? Qt::AlignLeft : Qt::AlignRight));
        } else {
            bool bottom = c->isOutput();
            rect = QRectF(pos + QPointF(-dx/2.0, bottom ? connector_radius_ : -connector_radius_-dy), QSize(dx, dy));
            opt = QTextOption(Qt::AlignVCenter | Qt::AlignCenter);
        }

        QColor color = c->isOutput() ? output_color_ : input_color_;
        QPen p = painter->pen();
        p.setColor(color.dark());
        painter->setPen(p);
//        painter->drawRect(rect);
        painter->drawText(rect, text, opt);
    }
}

bool DesignerScene::showConnectionContextMenu()
{
    QMenu menu;
    QAction* reset = new QAction("reset connection", &menu);
    menu.addAction(reset);
    QAction* del = new QAction("delete connection", &menu);
    menu.addAction(del);

    QAction* selectedItem = menu.exec(QCursor::pos());

    if(selectedItem == del) {
        dispatcher_->execute(graph_->deleteConnectionById(highlight_connection_id_));

    } else if(selectedItem == reset) {
        dispatcher_->execute(graph_->deleteAllConnectionFulcrumsCommand(highlight_connection_id_));
    }

    return true;
}

void DesignerScene::invalidateSchema()
{
    schema_dirty_ = true;
    update();
}

void DesignerScene::refresh()
{
    invalidateSchema();
}
