/// HEADER
#include <csapex/view/designer/designer_scene.h>

/// COMPONENT
#include <csapex/model/connectable.h>
#include <csapex/model/graph.h>
#include <csapex/view/widgets/port.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>
#include <csapex/model/connection.h>
#include <csapex/view/node/box.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/slot.h>
#include <csapex/signal/event.h>
#include <csapex/core/settings.h>
#include <csapex/command/dispatcher.h>
#include <csapex/command/command_factory.h>
#include <csapex/command/add_fulcrum.h>
#include <csapex/command/move_fulcrum.h>
#include <csapex/command/modify_fulcrum.h>
#include <csapex/view/designer/fulcrum_widget.h>
#include <csapex/view/widgets/movable_graphics_proxy_widget.h>
#include <csapex/utility/assert.h>
#include <csapex/model/fulcrum.h>
#include <csapex/view/widgets/message_preview_widget.h>
#include <csapex/model/graph_facade.h>
#include <csapex/command/delete_fulcrum.h>
#include <csapex/core/csapex_core.h>
#include <csapex/profiling/timer.h>
#include <csapex/profiling/profiler.h>
#include <csapex/profiling/interlude.hpp>

/// SYSTEM
#include <QtGui>
#include <QtOpenGL>

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

#define DEBUG_DRAWINGS_PER_SECOND 0

using namespace csapex;

const float DesignerScene::ARROW_LENGTH = 1.5f;

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

QPointF centerPoint(Port* port)
{
    apex_assert_hard(port);
    QPointF pos;
    QWidget* widget = port;
    QWidget* parent = nullptr;
    while (widget) {
        QGraphicsProxyWidget* proxy = widget->graphicsProxyWidget();
        if(proxy) {
            pos += proxy->pos();
        } else {
            pos += widget->pos();
        }
        parent = widget;
        widget = widget->parentWidget();
    }


    if(!port->isVisible()) {
        QSizeF s = 0.5 * QSizeF(parent->geometry().size());
        return parent->pos() + QPointF(s.width(), s.height());
    } else {
        QSizeF s = 0.5 * QSizeF(port->size());
        return pos + QPointF(s.width(), s.height());
    }
}

QPointF convert(const Point& p) {
    return QPointF(p.x, p.y);
}
Point convert(const QPointF& p) {
    return Point(p.x(), p.y());
}
}



DesignerScene::DesignerScene(GraphFacadePtr graph_facade, CsApexViewCore& view_core)
    : view_core_(view_core), graph_facade_(graph_facade),
      preview_(nullptr),
      draw_grid_(false), draw_schema_(false), display_messages_(true), display_signals_(true),
      scale_(1.0), highlight_connection_id_(-1), highlight_connection_sub_id_(-1), schema_dirty_(false),
      debug_(false)
{
    background_ = QPixmap::fromImage(QImage(":/background.png"));

    activity_marker_min_width_ = 3;
    activity_marker_max_width_ = 8;
    activity_marker_min_opacity_ = 50;
    activity_marker_max_opacity_ = 90;

    connector_radius_ = 7;

    setBackgroundBrush(QBrush(Qt::white));

    Graph* graph = graph_facade_->getGraph();

    connections_.push_back(graph->connection_added.connect([this](Connection* c) { connectionAdded(c); }));
    connections_.push_back(graph->connection_removed.connect([this](Connection* c) { connectionDeleted(c); }));

    for(const auto& connection : graph->getConnections()) {
        connectionAdded(connection.get());
    }
}

DesignerScene::~DesignerScene()
{
    for(auto c : connections_) {
        c.disconnect();
    }
    connections_.clear();
}

void DesignerScene::enableGrid(bool draw)
{
    if(draw != draw_grid_) {
        draw_grid_ = draw;

        invalidate();
    }
}

void DesignerScene::enableSchema(bool draw)
{
    if(draw != draw_schema_) {
        draw_schema_ = draw;

        invalidate();
    }
}

void DesignerScene::displayMessages(bool display)
{
    if(display != display_messages_) {
        display_messages_ = display;

        connection_bb_.clear();

        invalidate();
    }
}

void DesignerScene::enableDebug(bool debug)
{
    if(debug != debug_) {
        debug_ = debug;

        invalidate();
    }
}


void DesignerScene::displaySignals(bool display)
{
    if(display != display_signals_) {
        display_signals_ = display;

        connection_bb_.clear();

        invalidate();
    }
}

void DesignerScene::setScale(double scale)
{
    scale_ = scale;
    invalidateSchema();
}
void DesignerScene::drawBackground(QPainter *painter, const QRectF &rect)
{
    if(profiler_->isEnabled()) {
        profiler_->getTimer("drawBackground")->restart();
    }

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
        painter->setPen(QPen(QBrush(Qt::gray), 1./scale_));
        drawGrid(rect, painter, 10.0);

        painter->setPen(QPen(QBrush(Qt::gray), 2./scale_));
        drawGrid(rect, painter, 100.0);


        auto minx = rect.x();
        auto maxx = rect.x() + rect.width();
        auto miny = rect.y();
        auto maxy = rect.y() + rect.height();
        painter->setPen(QPen(QBrush(Qt::darkGray), 2./scale_));
        painter->drawLine(0, miny, 0, maxy);
        painter->drawLine(minx, 0, maxx, 0);
    }

    if(profiler_->isEnabled()) {
        profiler_->getTimer("drawBackground")->finish();
    }
}

void DesignerScene::drawGrid(const QRectF &rect, QPainter *painter, double dimension)
{
    auto w = rect.width();
    auto h = rect.height();

    double effective_dimension = dimension * scale_;
    if(effective_dimension < 5.0) {
        return;
    }

    auto minx = dimension * std::floor(rect.x() / dimension) - dimension;
    auto maxx = dimension * std::ceil((rect.x() + w) / dimension) + dimension;
    auto miny = dimension * std::floor(rect.y() / dimension) - dimension;
    auto maxy = dimension * std::ceil((rect.y() + h) / dimension) + dimension;

    for(auto x = minx; x <= maxx; x += dimension) {
        painter->drawLine(x, miny, x, maxy);
    }
    for(auto y = miny; y <= maxy; y += dimension) {
        painter->drawLine(minx, y, maxx, y);
    }
}


void DesignerScene::drawForeground(QPainter *painter, const QRectF &rect)
{
#if DEBUG_DRAWINGS_PER_SECOND
    static int drawings = 0;
    static long start = QDateTime::currentMSecsSinceEpoch();
    long now = QDateTime::currentMSecsSinceEpoch();
    long dt = now - start;
    ++drawings;
    if(dt > 0) {
        std::cerr << "drawing with avg. fps.: " << (drawings / (dt/1e3)) << std::endl;
    }
    //    auto print_rect = [](const QRectF& rect) {
    //        std::cerr << rect.x() << ", " << rect.y() << ", " << rect.x()+rect.width() << ", " << rect.y()+rect.height();
    //    };
    //    std::cerr << "rect is ";
    //    print_rect(rect);
    //    std::cerr << std::endl;

    long draw_begin = QDateTime::currentMSecsSinceEpoch();

#endif
    if(profiler_->isEnabled()) {
        profiling_timer_ = profiler_->getTimer("drawForeground");
        profiling_timer_->restart();
    } else {
        profiling_timer_.reset(new Timer("disabled"));
    }

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

    {
        INTERLUDE("connections");

        // check if we have temporary connections
        if(!temp_.empty()) {
            for(const TempConnection& temp : temp_) {
                ccs = CurrentConnectionState();

                if(temp.is_connected) {
                    drawConnection(painter, temp.from, temp.to_c, -1);

                } else {
                    Port* fromp = getPort(temp.from);

                    if(fromp) {
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
        }

        // draw all connections
        auto intersects_any = [](const std::vector<QRectF>& rects, const QRectF& rect) {
            for(auto r : rects) {
                if(rect.intersects(r)) {
                    return true;
                }
            }
            return false;
        };

        for(ConnectionPtr connection : graph_facade_->getGraph()->getConnections()) {
            auto pos = connection_bb_.find(connection.get());
            if(pos == connection_bb_.end() || intersects_any(pos->second, rect)) {
                drawConnection(painter, *connection);
            }
        }
    }

    // augment nodes

    {
        INTERLUDE("node augmentations");
        for(QGraphicsItem* item : items()) {
            MovableGraphicsProxyWidget* proxy = dynamic_cast<MovableGraphicsProxyWidget*>(item);
            if(!proxy) {
                continue;
            }

            NodeBox* box = proxy->getBox();

            if(!box || !rect.intersects(box->geometry())) {
                continue;
            }

            NodeHandle* node_handle = box->getNodeHandle();
            if(!node_handle) {
                continue;
            }


            // draw port information (in)
            for(auto input : node_handle->getExternalInputs()) {
                if(!node_handle->isParameterInput(input.get())) {
                    Port* p = getPort(input.get());
                    if(p) {
                        drawPort(painter, box->isSelected(), p);
                    }
                }
            }
            // draw port information (out)
            for(auto output : node_handle->getExternalOutputs()) {
                if(!node_handle->isParameterOutput(output.get())) {
                    Port* p = getPort(output.get());
                    if(p) {
                        drawPort(painter, box->isSelected(), p);
                    }
                }
            }

            // draw slots
            {
                int i = 0;
                for(auto slot : node_handle->getExternalSlots()) {
                    Port* p = getPort(slot.get());
                    if(p) {
                        drawPort(painter, box->isSelected(), p, i++);
                    }
                }
            }
            // draw events
            {
                int i = 0;
                for(auto event : node_handle->getExternalEvents()) {
                    Port* p = getPort(event.get());
                    if(p) {
                        drawPort(painter, box->isSelected(), p, i++);
                    }
                }
            }
        }
    }

    // augment graph ports
    {
        INTERLUDE("port augmentations");
        NodeHandle* nh = graph_facade_->getNodeHandle();
        if(nh) {
            {
                int i = 0;
                for(const InputPtr& input : nh->getInternalInputs()) {
                    Port* p = getPort(input.get());
                    if(p) {
                        drawPort(painter, false, p, i++);
                    }
                }
            }
            {
                int i = 0;
                for(const OutputPtr& output : nh->getInternalOutputs()) {
                    Port* p = getPort(output.get());
                    if(p) {
                        drawPort(painter, false, p, i++);
                    }
                }
            }
            {
                int i = 0;
                for(const SlotPtr& slot : nh->getInternalSlots()) {
                    Port* p = getPort(slot.get());
                    if(p) {
                        drawPort(painter, false, p, i++);
                    }
                }
            }
            {
                int i = 0;
                for(const EventPtr& event : nh->getInternalEvents()) {
                    Port* p = getPort(event.get());
                    if(p) {
                        drawPort(painter, false, p, i++);
                    }
                }
            }
        }
    }

    if(draw_schema_){
        painter->setOpacity(0.35);
        painter->drawImage(sceneRect().topLeft(), schematics);
    }

    if(debug_) {
        // draw outline
        painter->save();
        painter->setBrush(QBrush());
        painter->setPen(Qt::red);
        painter->drawRect(sceneRect());
        painter->restore();
    }

    schema_dirty_ = false;

    schematics_painter = nullptr;

#if DEBUG_DRAWINGS_PER_SECOND
    long draw_end = QDateTime::currentMSecsSinceEpoch();
    long dt_drawing = draw_end - draw_begin;
    std::cerr << "drawing took " << dt_drawing << "ms" << std::endl;
#endif

    if(profiler_->isEnabled()) {
        profiling_timer_->finish();
    }
}


void DesignerScene::mousePressEvent(QGraphicsSceneMouseEvent *e)
{
    QGraphicsScene::mousePressEvent(e);

    if(!e->isAccepted() && e->button() == Qt::LeftButton) {
        if(highlight_connection_id_ >= 0) {
            QPoint pos = e->scenePos().toPoint();
            view_core_.execute(Command::Ptr(new command::AddFulcrum(graph_facade_->getAbsoluteUUID(),
                                                                    highlight_connection_id_, highlight_connection_sub_id_,
                                                                    Point(pos.x(), pos.y()), Fulcrum::FULCRUM_LINEAR)));
            e->accept();

            // allow moving the fulcrum directly
            QGraphicsScene::mousePressEvent(e);
        }
    }
}

void DesignerScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *e)
{
    if(e->button() == Qt::MiddleButton && highlight_connection_id_ >= 0) {
        auto cmd = CommandFactory(graph_facade_.get()).deleteConnectionByIdCommand(highlight_connection_id_);
        if(cmd) {
            view_core_.execute(cmd);
        }
        return;
    }

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

    auto* item = itemAt(e->scenePos(), QTransform());
    Port* port = nullptr;
    if(item && item->type() == QGraphicsProxyWidget::Type) {
        QGraphicsProxyWidget* proxy = static_cast<QGraphicsProxyWidget*>(item);
        QWidget* widget = proxy->widget();
        if(!widget) {
            return;
        }
        QPointF p = proxy->mapFromScene(e->scenePos());
        QWidget* child = widget->childAt(p.toPoint());

        port = dynamic_cast<Port*>(child);
    }

    if(!port && data.first >= 0) {
        if(data.first != highlight_connection_id_) {
            highlight_connection_id_ = data.first;
            highlight_connection_sub_id_ = data.second;
        }

        auto c = graph_facade_->getGraph()->getConnectionWithId(highlight_connection_id_);
        if(!c) {
            return;
        }
        auto m = c->getToken();
        if(debug_){
            QString descr("Connection #");
            descr += QString::number(c->id());
            descr += " (";
            descr += QString::fromStdString(c->from()->getUUID().getShortName());
            descr += " -> ";
            descr += QString::fromStdString(c->to()->getUUID().getShortName());
            descr += "), state: ";

            switch (c->getState()) {
            case Connection::State::DONE:
                descr += "NOT_INITIALIZED / DONE";
                break;
            case Connection::State::UNREAD:
                descr += "UNREAD";
                break;
            case Connection::State::READ:
                descr += "READ";
                break;
            default:
                break;
            }

            if(m) {
                descr += ", Message: ";
                descr += QString::fromStdString(m->getTokenData()->descriptiveName());
                descr += ", # " + QString::number(m->getSequenceNumber());
            }

            descr += ")";

            for(auto v : views()) {
                v->setToolTip(descr);
            }
        }

        QPointF preview_pos = QCursor::pos() + QPointF(20, 20);

        if(!preview_) {
            preview_ = new MessagePreviewWidget;
            preview_->hide();
        }

        preview_->setWindowTitle(QString::fromStdString("Output"));
        preview_->move(preview_pos.toPoint());

        if(!preview_->isConnected()) {
            preview_->connectTo(c->from());
        }

        update();

    } else if(highlight_connection_id_ >= 0)  {
        highlight_connection_id_ = -1;
        for(auto v : views()) {
            v->setToolTip("");
        }

        if(preview_) {
            preview_->disconnect();
            preview_->hide();
            preview_->deleteLater();
            preview_ = nullptr;
        }
        update();
    }
}

int DesignerScene::getHighlightedConnectionId() const
{
    return highlight_connection_id_;
}

bool DesignerScene::isEmpty() const
{
    return graph_facade_->getGraph()->countNodes() == 0;
}

void DesignerScene::connectionAdded(Connection* c)
{
    for(FulcrumPtr f : c->getFulcrums()) {
        fulcrumAdded(f.get());
    }

    c->fulcrum_added.connect(std::bind(&DesignerScene::fulcrumAdded, this, std::placeholders::_1));
    c->fulcrum_deleted.connect(std::bind(&DesignerScene::fulcrumDeleted, this, std::placeholders::_1));
    c->fulcrum_moved.connect(std::bind(&DesignerScene::fulcrumMoved, this, std::placeholders::_1, std::placeholders::_2));
    c->fulcrum_moved_handle.connect(std::bind(&DesignerScene::fulcrumHandleMoved, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    c->fulcrum_type_changed.connect(std::bind(&DesignerScene::fulcrumTypeChanged, this, std::placeholders::_1, std::placeholders::_2));

    invalidateSchema();
}

void DesignerScene::connectionDeleted(Connection*)
{
    invalidateSchema();
}

void DesignerScene::fulcrumAdded(Fulcrum * f)
{
    std::map<Fulcrum*, FulcrumWidget*>::iterator pos = fulcrum_2_widget_.find(f);
    if(pos != fulcrum_2_widget_.end()) {
        return;
    }

    FulcrumWidget* w = new FulcrumWidget(f);
    addItem(w);
    fulcrum_2_widget_[f] = w;
    fulcrum_last_pos_[f] = f->pos();
    fulcrum_last_type_[f] = f->type();
    fulcrum_last_hin_[f] = f->handleIn();
    fulcrum_last_hout_[f] = f->handleOut();

    QObject::connect(w, &FulcrumWidget::deleteRequest, [this](Fulcrum* f){
        view_core_.execute(Command::Ptr(new command::DeleteFulcrum(graph_facade_->getAbsoluteUUID(), f->connectionId(), f->id())));
    });



    QObject::connect(w, &FulcrumWidget::modifyRequest, [this](Fulcrum* f, int type){
        command::ModifyFulcrum::Ptr cmd(new command::ModifyFulcrum(
                                            graph_facade_->getAbsoluteUUID(),
                                            f->connectionId(), f->id(),
                                            f->type(), f->handleIn(), f->handleOut(),
                                            type, f->handleIn(), f->handleOut()));
        view_core_.execute(cmd);
    });

    clearSelection();
    w->setSelected(true);
    setFocusItem(w, Qt::MouseFocusReason);

    invalidateSchema();
}

void DesignerScene::fulcrumDeleted(void *fulcrum)
{
    Fulcrum* f = (Fulcrum*) fulcrum;

    std::map<Fulcrum*, FulcrumWidget*>::iterator pos = fulcrum_2_widget_.find(f);
    if(pos == fulcrum_2_widget_.end()) {
        return;
    }

    //    delete pos->second;
    pos->second->deleteLater();
    fulcrum_2_widget_.erase(pos);

    invalidateSchema();
}

void DesignerScene::fulcrumMoved(void * fulcrum, bool dropped)
{
    Fulcrum* f = (Fulcrum*) fulcrum;

    if(dropped) {
        view_core_.execute(Command::Ptr(new command::MoveFulcrum(graph_facade_->getAbsoluteUUID(), f->connectionId(), f->id(), fulcrum_last_pos_[f], f->pos())));
        fulcrum_last_pos_[f] = f->pos();
    }
    invalidateSchema();
}

void DesignerScene::fulcrumHandleMoved(void * fulcrum, bool dropped, int /*which*/)
{
    Fulcrum* f = (Fulcrum*) fulcrum;

    if(dropped) {
        view_core_.execute(Command::Ptr(new command::ModifyFulcrum(graph_facade_->getAbsoluteUUID(), f->connectionId(), f->id(),
                                                                   fulcrum_last_type_[f], fulcrum_last_hin_[f], fulcrum_last_hout_[f],
                                                                   f->type(), f->handleIn(), f->handleOut())));
        fulcrum_last_type_[f] = f->type();
        fulcrum_last_hin_[f] = f->handleIn();
        fulcrum_last_hout_[f] = f->handleOut();

    }
    invalidateSchema();
}

void DesignerScene::fulcrumTypeChanged(void */*f*/, int /*type*/)
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

    ConnectablePtr from = connection.from();
    ConnectablePtr to = connection.to();

    int id = connection.id();

    ccs = CurrentConnectionState();

    ccs.disabled = !(connection.isSourceEnabled() && connection.isSinkEnabled());
    ccs.full_read = connection.getState() == Connection::State::READ;
    ccs.full_unread = connection.getState() == Connection::State::UNREAD;
    ccs.active = connection.isActive();
    ccs.active_token = connection.holdsActiveToken();
    if(NodeHandlePtr node = std::dynamic_pointer_cast<NodeHandle>(to->getOwner())) {
        ccs.target_is_pipelining = node->getNodeState()->getExecutionMode() == ExecutionMode::PIPELINING;

    } else {
        ccs.target_is_pipelining = false;
    }

    if(debug_){
        ccs.label = QString::number(connection.from()->sequenceNumber()) +
                " -> " +
                QString::number(connection.to()->sequenceNumber());
    }
    connection_bb_[&connection] = drawConnection(painter, from.get(), to.get(), id);
}

std::vector<QRectF> DesignerScene::drawConnection(QPainter *painter,
                                                  Connectable *from, Connectable *to,
                                                  int id)
{
    Port* from_port = getPort(from);
    Port* to_port = getPort(to);

    if(!from_port || !to_port) {
        return std::vector<QRectF>();
    }

    if(dynamic_cast<Event*>(from) != nullptr) {
        if(!display_signals_) {
            return std::vector<QRectF>();
        }
        ccs.type = TokenType::SIG;

    } else {
        if(!display_messages_) {
            return std::vector<QRectF>();
        }
        ccs.type = TokenType::MSG;
    }


    QPointF p1 = centerPoint(from_port);
    QPointF p2 = centerPoint(to_port);

    ccs.highlighted = (highlight_connection_id_ == id);
    ccs.error = (to->isError() || from->isError());
    ccs.minimized_from = from_port->isMinimizedSize();
    ccs.minimized_to = to_port->isMinimizedSize();
    ccs.hidden_from = !from_port->isVisible();
    ccs.hidden_to = !to_port->isVisible();
    ccs.selected_from = from_port->property("focused").toBool();
    ccs.selected_to = to_port->property("focused").toBool();

    if(dynamic_cast<Event*>(from)) {
        ccs.start_pos = BOTTOM;
    } else {
        ccs.start_pos = from_port->isFlipped() ? LEFT : RIGHT;
    }
    if(dynamic_cast<Slot*>(to)) {
        ccs.end_pos = TOP;
    } else {
        ccs.end_pos = to_port->isFlipped() ? RIGHT : LEFT;
    }

    return drawConnection(painter, p1, p2, id);
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


std::vector<QRectF> DesignerScene::drawConnection(QPainter *painter, const QPointF& from, const QPointF& real_to, int id)
{
    QPointF to = offset(real_to, ccs.end_pos, view_core_.getStyle().lineWidth()*ARROW_LENGTH);

    painter->setRenderHint(QPainter::Antialiasing);

    double scale_factor = 1.0 / scale_;
    if(scale_factor < 1.0) {
        scale_factor = 1.0;
    }

    if(ccs.type == TokenType::SIG) {
        scale_factor *= 0.5;
    }

    if(ccs.active) {
        scale_factor *= 3;
    }

    ccs.minimized = ccs.minimized_from || ccs.minimized_to;
    ccs.r = ccs.minimized ? view_core_.getStyle().lineWidth() / 2.0 : view_core_.getStyle().lineWidth();
    ccs.r *= scale_factor;

    double max_slack_height = 40.0;
    double mindist_for_slack = 60.0;
    double slack_smooth_distance = 300.0;

    Fulcrum::Ptr first(new Fulcrum(nullptr, convert(from), Fulcrum::FULCRUM_OUT, convert(from), convert(from)));
    Fulcrum::Ptr current = first;
    Fulcrum::Ptr last = current;

    std::vector<Fulcrum::Ptr> targets;
    if(id >= 0) {
        ConnectionPtr connection = graph_facade_->getGraph()->getConnectionWithId(id);
        targets = connection->getFulcrums();
    }
    targets.push_back(Fulcrum::Ptr(new Fulcrum(nullptr, convert(to), Fulcrum::FULCRUM_IN, convert(to), convert(to))));

    int sub_section = 0;

    QPointF cp1, cp2;

    // paths
    typedef std::pair<QPainterPath, int> Path;
    std::vector<Path> paths;

    // generate lines
    for(std::size_t i = 0; i < targets.size(); ++i) {
        const Fulcrum::Ptr& next = targets[i];

        QPointF current_pos = convert(current->pos());
        QPointF next_pos = convert(next->pos());

        QPointF diff = (next_pos - current_pos);
        double direct_length = hypot(diff.x(), diff.y());

        QPoint y_offset;
        double x_offset = 0;
        if(direct_length > mindist_for_slack) {
            double offset_factor = std::min(1.0, (direct_length - mindist_for_slack) / slack_smooth_distance);

            x_offset = std::max(offset_factor * mindist_for_slack, std::abs(0.45 * (diff).x()));
            y_offset = QPoint(0, offset_factor * max_slack_height);
        }

        QPainterPath path(current_pos);

        if(current->type() == Fulcrum::FULCRUM_OUT) {
            cp1 = offset(current_pos, ccs.start_pos, x_offset) + y_offset;

        } else if(current->type() == Fulcrum::FULCRUM_IN) {
            cp1 = offset(current_pos, ccs.end_pos, x_offset) + y_offset;

        } else {
            const Fulcrum::Ptr& last = targets[sub_section-1];
            if(last->type() == Fulcrum::FULCRUM_LINEAR) {
                cp1 = current_pos;
            } else {
                cp1 = current_pos + convert(current->handleOut());
            }
        }

        if(next->type() == Fulcrum::FULCRUM_OUT) {
            cp2 = offset(next_pos, ccs.start_pos, x_offset) + y_offset;

        } else if(next->type() == Fulcrum::FULCRUM_IN) {
            cp2 = offset(next_pos, ccs.end_pos, x_offset) + y_offset;

        } else {
            if(next->type() == Fulcrum::FULCRUM_LINEAR) {
                cp2 = next_pos;
            } else {
                cp2 = next_pos + convert(next->handleIn());
            }
        }

        path.cubicTo(cp1, cp2, next_pos);

        paths.push_back(std::make_pair(path, sub_section));

        last = current;
        current = next;
        ++sub_section;
    }

    // arrow
    QPolygonF arrow;
    QPointF a = offset(QPointF(0,0), ccs.end_pos, ARROW_LENGTH * view_core_.getStyle().lineWidth()) * scale_factor;

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
        for(const Path& path : paths) {
            painter->drawPath(path.first);
        }
        painter->drawPath(arrow_path);

        painter->setPen(QPen(Qt::white, ccs.r + 3*scale_factor, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        for(const Path& path : paths) {
            painter->drawPath(path.first);
        }
        painter->drawPath(arrow_path);
    }

    QColor color_start = view_core_.getStyle().lineColor();
    QColor color_end = view_core_.getStyle().lineColor();

    if(ccs.full_read || ccs.full_unread) {
        color_start = view_core_.getStyle().lineColorBlocked();
        color_end = view_core_.getStyle().lineColorBlocked();

        if(ccs.full_read) {
            color_start = color_start.dark();
            color_end = color_end.dark();
        }

    } else if(ccs.error) {
        color_start = view_core_.getStyle().lineColorError();
        color_end = view_core_.getStyle().lineColorError();

    } else if(ccs.disabled) {
        color_start = view_core_.getStyle().lineColorDisabled();
        color_end = view_core_.getStyle().lineColorDisabled();

    }
    if(ccs.selected_from) {
        color_start.setAlpha(255);
    } else {
        color_start.setAlpha(100);
    }

    if(ccs.selected_to) {
        color_end.setAlpha(255);
    }else {
        color_end.setAlpha(100);
    }

    if(ccs.hidden_from) {
        color_start.setAlpha(60);
    }
    if(ccs.hidden_to) {
        color_end.setAlpha(60);
    }

    if(ccs.active_token) {
        color_start.setAlpha(255);
        color_end.setAlpha(255);
    }

    QLinearGradient lg(from, to);
    lg.setColorAt(0, color_start);
    lg.setColorAt(1, color_end);

    painter->setPen(QPen(QBrush(lg), ccs.r * 0.75,
                         ccs.target_is_pipelining ? Qt::DotLine : Qt::SolidLine,
                         ccs.target_is_pipelining ? Qt::SquareCap: Qt::RoundCap,
                         Qt::RoundJoin));

    std::vector<QRectF> bounding_boxes;

    for(const Path& path : paths) {
        painter->drawPath(path.first);
        bounding_boxes.push_back(path.first.boundingRect());

        if(id >= 0 && schema_dirty_) {
            QPen schema_pen = QPen(QColor(id2rgb(id, path.second)), ccs.r * 1.75, Qt::SolidLine, Qt::RoundCap,Qt::RoundJoin);
            schematics_painter->setPen(schema_pen);
            schematics_painter->drawPath(path.first);
        }
    }
    painter->setBrush(color_end);
    painter->setPen(QPen(painter->brush(), 1.0));
    painter->drawPath(arrow_path);

    bounding_boxes.push_back(arrow_path.boundingRect());

    if(draw_schema_) {
        painter->setBrush(QBrush());
        for(auto r: bounding_boxes) {
            painter->drawRect(r);
        }
    }

    painter->drawText(QPointF(from + real_to) * 0.5, ccs.label);

    return bounding_boxes;
}

void DesignerScene::addPort(Port *port)
{
    ConnectablePtr c = port->getAdaptee().lock();
    if(c) {
        port_map_[c->getUUID()] = port;
    }
}

void DesignerScene::removePort(Port *port)
{
    if(!port_map_.empty()) {
        for(auto it = port_map_.begin(); it != port_map_.end(); ) {
            if(it->second == port) {
                it = port_map_.erase(it);
            } else {
                ++it;
            }
        }
    }
}

Port* DesignerScene::getPort(Connectable *c)
{
    auto pos = port_map_.find(c->getUUID());
    if(pos != port_map_.end()) {
        return pos->second;
    } else {
        return nullptr;
    }
}

void DesignerScene::drawPort(QPainter *painter, bool selected, Port *p, int pos)
{
    INTERLUDE("drawPort");
    auto* item = itemAt(centerPoint(p), QTransform());
    if(!item || item->type() != QGraphicsProxyWidget::Type) {
        return;
    }

    QGraphicsProxyWidget* proxy = static_cast<QGraphicsProxyWidget*>(item);
    QWidget* widget = proxy->widget();
    if(!widget) {
        return;
    }
    QPointF pt = proxy->mapFromScene(centerPoint(p));
    QWidget* child = widget->childAt(pt.toPoint());

    if(dynamic_cast<Port*>(child) != p) {
        return;
    }

    ConnectablePtr c = p->getAdaptee().lock();
    if(!c) {
        return;
    }

    bool is_message = (dynamic_cast<Slot*>(c.get()) == nullptr && dynamic_cast<Event*>(c.get()) == nullptr);

    if(!p->isMinimizedSize())  {
        INTERLUDE("overlays");

        int font_size = debug_ ? 10 : 8;
        int lines = 1;


        painter->save();

        QFont font("Ubuntu Condensed, Liberation Sans, FreeSans, Arial", font_size, QFont::Light);
        painter->setFont(font);

        QString text;

        if(debug_) {
            text = "Seq: ";
            text += QString::number(c->sequenceNumber());
            text += "\nCount: ";
            text += QString::number(c->getCount());

        } else {
            text = QString::fromStdString(c->getLabel());

            if(is_message) {
                QString descr = QString::fromStdString(c->getType()->descriptiveName());
                if(!descr.isEmpty()) {
                    if(text.length() != 0) {
                        text += '\n';
                    }
                    text += descr;
                    ++lines;
                }
            }
        }


        QFontMetrics metrics(font);

        QRect no_rect;
        QRectF rect = metrics.boundingRect(no_rect, Qt::AlignLeft | Qt::AlignTop, text);
        rect.translate(centerPoint(p));

        int dx = rect.width();
        int dy = rect.height();

        QTextOption opt;
        opt.setUseDesignMetrics(true);
        opt.setWrapMode(QTextOption::WrapAnywhere);

        enum class Direction {
            UP, RIGHT, DOWN, LEFT
        };;

        Direction dir = Direction::LEFT;

        if(is_message) {
            dir = (c->isOutput() ^ p->isFlipped()) ? Direction::LEFT : Direction::RIGHT;
            rect.translate((dir == Direction::LEFT) ? 2*connector_radius_ : -2*connector_radius_-dx, -dy / 2.0);
            opt = QTextOption(Qt::AlignVCenter | ((dir == Direction::LEFT) ? Qt::AlignLeft : Qt::AlignRight));
        } else {
            dir = c->isOutput() ? Direction::UP : Direction::DOWN;
            auto distance = connector_radius_ + 3;
            rect.translate(-dx/2.0, (dir == Direction::UP) ? distance : -distance-dy);
        }

        // draw every other port offset
        if(pos >= 0 && ((pos % 2) == 1)) {
            if(dir == Direction::UP || dir == Direction::DOWN) {
                rect.translate(0, dir == Direction::UP ? metrics.height() : -metrics.height());
            } else {
                rect.translate(0, 0);
            }
        }

        INTERLUDE("overlay drawing");
        // drawing
        QColor box_color = view_core_.getStyle().balloonColor();
        QColor text_color = view_core_.getStyle().lineColor();

        if(p->isHovered()) {
            box_color.setAlphaF(0.8);
        } else {
            box_color.setAlphaF(selected ? 0.4 : 0.1);
        }

        // draw box
        QPainterPath path;

        QPointF tl = rect.topLeft() + QPointF(-2, 0);
        QPointF bl = rect.bottomLeft() + QPointF(-2, 0);
        QPointF tr = rect.topRight() + QPointF(2, 0);
        QPointF br = rect.bottomRight() + QPointF(2, 0);

        double corner = 5.0;
        QSizeF corner_size(corner * 2, corner * 2);
        QPointF offset_x(corner, 0);
        QPointF offset_y(0, corner);

        QPointF port_center = centerPoint(p);

        auto drawSide = [&port_center, &path]
                (const Direction& dir, const Direction& side,
                const QPointF& start, const QPointF& end, QPointF end_offset)
        {
            QPointF real_end = end + end_offset;
            if(dir == side) {
                QPointF cp = 0.5 * (start + end);
                path.cubicTo(cp, port_center, port_center);
                path.cubicTo(port_center, cp, real_end);
            } else {
                path.lineTo(real_end);
            }
        };

        path.moveTo(tl + offset_x);
        path.arcTo(QRectF(tl, corner_size), 90.0, 90.0);
        drawSide(dir, Direction::LEFT, tl, bl, -offset_y);
        path.arcTo(QRectF(bl - 2*offset_y, corner_size), 180.0, 90.0);
        drawSide(dir, Direction::DOWN, bl, br, -offset_x);
        path.arcTo(QRectF(br - 2* offset_x - 2*offset_y, corner_size), -90.0, 90.0);
        drawSide(dir, Direction::RIGHT, br, tr, offset_y);
        path.arcTo(QRectF(tr - 2* offset_x, corner_size), 0.0, 90.0);
        drawSide(dir, Direction::UP, tr, tl, offset_x);

        painter->setBrush(QBrush(box_color));
        painter->setPen(QPen(box_color.dark()));
        painter->drawPath(path);

        // draw text
        QPen pen = painter->pen();
        pen.setColor(text_color.dark());
        painter->setBrush(QBrush());
        painter->setPen(pen);
        painter->drawText(rect, text, opt);

        painter->restore();
    }
}

bool DesignerScene::showConnectionContextMenu()
{
    ConnectionConstPtr c = graph_facade_->getGraph()->getConnectionWithId(highlight_connection_id_);

    QMenu menu;
    QAction* reset = new QAction("reset connection", &menu);
    menu.addAction(reset);
    QAction* del = new QAction("delete connection", &menu);
    menu.addAction(del);

    menu.addSeparator();

    QAction* active = new QAction("allow active tokens", &menu);
    active->setCheckable(true);
    active->setChecked(c->isActive());
    menu.addAction(active);

    QAction* selectedItem = menu.exec(QCursor::pos());

    if(selectedItem == del) {
        view_core_.execute(CommandFactory(graph_facade_.get()).deleteConnectionByIdCommand(highlight_connection_id_));

    } else if(selectedItem == reset) {
        view_core_.execute(CommandFactory(graph_facade_.get()).deleteAllConnectionFulcrumsCommand(highlight_connection_id_));

    } else if(selectedItem == active) {
        view_core_.execute(CommandFactory(graph_facade_.get()).setConnectionActive(highlight_connection_id_, active->isChecked()));
    }


    return true;
}

void DesignerScene::setSelection(const NodeBox *box)
{
    clearSelection();

    for(QGraphicsItem* item : items()) {
        MovableGraphicsProxyWidget* proxy = dynamic_cast<MovableGraphicsProxyWidget*>(item);
        if(proxy && proxy->getBox() == box) {
            proxy->setSelected(true);
        }
    }

    QApplication::processEvents();
}

std::vector<NodeBox*> DesignerScene::getSelectedBoxes() const
{
    std::vector<NodeBox*> r;
    for(QGraphicsItem* item : selectedItems()) {
        MovableGraphicsProxyWidget* proxy = dynamic_cast<MovableGraphicsProxyWidget*>(item);
        if(proxy) {
            r.push_back(proxy->getBox());
        }
    }
    return r;
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

std::string DesignerScene::makeStatusString() const
{
    std::stringstream ss;
    ss << "Temporary connections: " << temp_.size() << '\n';
    if(!temp_.empty()) {
        for(const TempConnection& c : temp_) {
            if(c.is_connected) {
                ss << " - " << c.from->getUUID() << " => " << c.to_c->getUUID() << '\n';
            } else {
                ss << " - " << c.from->getUUID() << " => [" << c.to_p.x() << ", " << c.to_p.y() << "]\n";
            }
        }
    }
    return ss.str();
}

/// MOC
#include "../../../include/csapex/view/designer/moc_designer_scene.cpp"
