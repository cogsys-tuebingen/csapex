/// HEADER
#include <csapex/view/designer/designer_scene.h>

/// COMPONENT
#include <csapex/command/add_fulcrum.h>
#include <csapex/command/command_factory.h>
#include <csapex/command/delete_fulcrum.h>
#include <csapex/command/dispatcher.h>
#include <csapex/command/modify_fulcrum.h>
#include <csapex/command/move_fulcrum.h>
#include <csapex/core/settings.h>
#include <csapex/model/connection.h>
#include <csapex/model/connector.h>
#include <csapex/model/fulcrum.h>
#include <csapex/model/graph_facade.h>
#include <csapex/model/graph_facade_impl.h>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/model/graph.h>
#include <csapex/msg/marker_message.h>
#include <csapex/profiling/interlude.hpp>
#include <csapex/profiling/profiler.h>
#include <csapex/profiling/timer.h>
#include <csapex/utility/assert.h>
#include <csapex/view/designer/fulcrum_widget.h>
#include <csapex/view/node/box.h>
#include <csapex/view/widgets/message_preview_widget.h>
#include <csapex/view/widgets/movable_graphics_proxy_widget.h>
#include <csapex/view/widgets/port.h>

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
      draw_grid_(false), draw_schema_(false),
      display_messages_(true), display_signals_(true),
      display_active_(true), display_inactive_(true),
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

    connections_.push_back(graph_facade_->connection_added.connect([this](const ConnectionDescription& ci) {
        connectionAdded(ci);
    }));
    connections_.push_back(graph_facade_->connection_removed.connect([this](const ConnectionDescription& ci) {
        connectionDeleted(ci);
    }));

    for(const ConnectionDescription& ci : graph_facade_->enumerateAllConnections()){
        connectionAdded(ci);
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
    displayConnections(display_messages_, display);
}
void DesignerScene::displaySignals(bool display)
{
    displayConnections(display_signals_, display);
}
void DesignerScene::displayActive(bool display)
{
    displayConnections(display_active_, display);
}
void DesignerScene::displayInactive(bool display)
{
    displayConnections(display_inactive_, display);
}

void DesignerScene::displayConnections(const QString& type, bool display)
{
    if(type == "messages") {
        displayMessages(display);
    } else if(type == "signals") {
        displaySignals(display);
    } else if(type == "active") {
        displayActive(display);
    } else if(type == "inactive") {
        displayInactive(display);
    }
}


void DesignerScene::displayConnections(bool &member, bool display)
{
    if(display != member) {
        member = display;

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
                    ConnectorPtr temp_from = temp.from.lock();
                    ConnectorPtr temp_to = temp.to_c.lock();
                    if(temp_from && temp_to) {
                        drawConnection(painter, temp_from.get(), temp_to.get(), -1);
                    }

                } else {
                    ConnectorPtr temp_from = temp.from.lock();

                    if(temp_from) {
                        Port* fromp = getPort(temp_from->getUUID());

                        if(fromp) {
                            ccs.start_pos = UNDEFINED;
                            ccs.end_pos = UNDEFINED;


                            if(temp_from->isInput()) {
                                drawConnection(painter, temp.to_p, centerPoint(fromp), -1);
                            } else {
                                drawConnection(painter, centerPoint(fromp), temp.to_p, -1);
                            }
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

        for(const ConnectionDescription& connection : graph_facade_->enumerateAllConnections()) {
            auto pos = connection_bb_.find(connection.id);
            if(pos == connection_bb_.end() || intersects_any(pos->second, rect)) {
                drawConnection(painter, connection);
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

            NodeFacadePtr node_facade = box->getNodeFacade();

            // draw port information (in)
            for(const ConnectorDescription& input : node_facade->getInputs()) {
                if(!node_facade->isParameterInput(input.id)) {
                    Port* p = getPort(input.id);
                    if(p) {
                        drawPort(painter, box->isSelected(), p);
                    }
                }
            }
            // draw port information (out)
            for(const ConnectorDescription& output : node_facade->getOutputs()) {
                if(!node_facade->isParameterOutput(output.id)) {
                    Port* p = getPort(output.id);
                    if(p) {
                        drawPort(painter, box->isSelected(), p);
                    }
                }
            }

            // draw slots
            {
                int i = 0;
                for(const ConnectorDescription& slot : node_facade->getSlots()) {
                    Port* p = getPort(slot.id);
                    if(p) {
                        drawPort(painter, box->isSelected(), p, i++);
                    }
                }
            }
            // draw events
            {
                int i = 0;
                for(const ConnectorDescription& event : node_facade->getEvents()) {
                    Port* p = getPort(event.id);
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
        NodeFacadePtr nf = graph_facade_->getNodeFacade();
        if(nf) {
            {
                int i = 0;
                for(const ConnectorDescription& input : nf->getInternalInputs()) {
                    Port* p = getPort(input.id);
                    if(p) {
                        drawPort(painter, false, p, i++);
                    }
                }
            }
            {
                int i = 0;
                for(const ConnectorDescription& output : nf->getInternalOutputs()) {
                    Port* p = getPort(output.id);
                    if(p) {
                        drawPort(painter, false, p, i++);
                    }
                }
            }
            {
                int i = 0;
                for(const ConnectorDescription& slot : nf->getInternalSlots()) {
                    Port* p = getPort(slot.id);
                    if(p) {
                        drawPort(painter, false, p, i++);
                    }
                }
            }
            {
                int i = 0;
                for(const ConnectorDescription& event : nf->getInternalEvents()) {
                    Port* p = getPort(event.id);
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
            view_core_.getCommandDispatcher()->execute(Command::Ptr(new command::AddFulcrum(graph_facade_->getAbsoluteUUID(),
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
            view_core_.getCommandDispatcher()->execute(cmd);
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

        try {
            const ConnectionDescription& c = graph_facade_->getConnectionWithId(highlight_connection_id_);

            if(debug_){
                QString descr("Connection #");
                descr += QString::number(c.id);
                descr += " (";
                descr += QString::fromStdString(c.from.getShortName());
                descr += " -> ";
                descr += QString::fromStdString(c.to.getShortName());
                descr += ") ";

                TokenDataConstPtr m = c.type;
                if(m) {
                    descr += ", Message: ";
                    descr += QString::fromStdString(m->descriptiveName());
                    //                descr += ", # " + QString::number(m->getSequenceNumber());
                } else {
                    descr += " (no message)";
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
                preview_->connectTo(graph_facade_->findConnector(c.from));
            }

            update();

        } catch(const std::exception& e) {
            std::cerr << "Error handling connection: " << e.what() << std::endl;
        }

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
    return graph_facade_->countNodes() == 0;
}

void DesignerScene::connectionAdded(const ConnectionDescription& ci)
{
    for(const Fulcrum& f : ci.fulcrums) {
        connection_2_fulcrum_[ci.id].push_back(f);
        Fulcrum* proxy = &connection_2_fulcrum_[ci.id].back();
        fulcrumAdded(proxy);
    }


    if(GraphFacadeImplementationPtr gfl = std::dynamic_pointer_cast<GraphFacadeImplementation>(graph_facade_)) {
        ConnectionPtr connection = gfl->getLocalGraph()->getConnection(ci.from, ci.to);

        connection->fulcrum_added.connect(
                    std::bind(&DesignerScene::fulcrumAdded, this, std::placeholders::_1));
        connection->fulcrum_deleted.connect(
                    std::bind(&DesignerScene::fulcrumDeleted, this, std::placeholders::_1));
        connection->fulcrum_moved.connect(
                    std::bind(&DesignerScene::fulcrumMoved, this, std::placeholders::_1, std::placeholders::_2));
        connection->fulcrum_moved_handle.connect(
                    std::bind(&DesignerScene::fulcrumHandleMoved, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        connection->fulcrum_type_changed.connect(
                    std::bind(&DesignerScene::fulcrumTypeChanged, this, std::placeholders::_1, std::placeholders::_2));

    } else {
        // TODO: implement fulcrum manipulation for remote connections!
    }

    invalidateSchema();
}

void DesignerScene::connectionDeleted(const ConnectionDescription& ci)
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
        view_core_.getCommandDispatcher()->execute(Command::Ptr(new command::DeleteFulcrum(graph_facade_->getAbsoluteUUID(), f->connectionId(), f->id())));
    });



    QObject::connect(w, &FulcrumWidget::modifyRequest, [this](Fulcrum* f, int type){
        command::ModifyFulcrum::Ptr cmd(new command::ModifyFulcrum(
                                            graph_facade_->getAbsoluteUUID(),
                                            f->connectionId(), f->id(),
                                            f->type(), f->handleIn(), f->handleOut(),
                                            type, f->handleIn(), f->handleOut()));
        view_core_.getCommandDispatcher()->execute(cmd);
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
        view_core_.getCommandDispatcher()->execute(Command::Ptr(new command::MoveFulcrum(graph_facade_->getAbsoluteUUID(), f->connectionId(), f->id(), fulcrum_last_pos_[f], f->pos())));
        fulcrum_last_pos_[f] = f->pos();
    }
    invalidateSchema();
}

void DesignerScene::fulcrumHandleMoved(void * fulcrum, bool dropped, int /*which*/)
{
    Fulcrum* f = (Fulcrum*) fulcrum;

    if(dropped) {
        view_core_.getCommandDispatcher()->execute(Command::Ptr(new command::ModifyFulcrum(graph_facade_->getAbsoluteUUID(), f->connectionId(), f->id(),
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

void DesignerScene::addTemporaryConnection(ConnectorPtr from, const QPointF& end)
{
    apex_assert_hard(from);

    TempConnection temp(false);
    temp.from = from;
    temp.to_p = end;

    temp_.push_back(temp);

    update();
}

void DesignerScene::previewConnection(ConnectorPtr from, ConnectorPtr to)
{
    addTemporaryConnection(from, to);
    update();
}

void DesignerScene::addTemporaryConnection(ConnectorPtr from, ConnectorPtr to)
{
    apex_assert_hard(from);
    apex_assert_hard(to);

    ConnectorPtr input;
    ConnectorPtr output;
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

void DesignerScene::drawConnection(QPainter *painter, const ConnectionDescription& ci)
{
    if(ci.active && !display_active_) {
        return;
    }
    if(!ci.active && !display_inactive_) {
        return;
    }

    ConnectorPtr from = graph_facade_->findConnector(ci.from);
    ConnectorPtr to = graph_facade_->findConnector(ci.to);

    ccs = CurrentConnectionState();

    //    bool marker = false;
    //    TokenPtr token = connection.getToken();
    //    if(token) {
    //        marker = std::dynamic_pointer_cast<connection_types::MarkerMessage const>(token->getTokenData()) != nullptr;
    //    }

    if(GraphFacadeImplementationPtr gfl = std::dynamic_pointer_cast<GraphFacadeImplementation>(graph_facade_)) {
        ConnectionPtr connection = gfl->getLocalGraph()->getConnection(ci.from, ci.to);

        ccs.disabled = !(from->isEnabled() && to->isEnabled());
        ccs.full_read = connection->getState() == Connection::State::READ;
        ccs.full_unread = connection->getState() == Connection::State::UNREAD;
        ccs.active = connection->isActive();
        ccs.active_token = connection->holdsActiveToken();
        ccs.target_is_pipelining = connection->isPipelining();

    } else {
        ccs.disabled = !(from->isEnabled() && to->isEnabled());
        // TODO: implement state forwarding for connections
        ccs.full_read = false;
        ccs.full_unread = false;
        ccs.active = ci.active;
        ccs.active_token = false;
        ccs.target_is_pipelining = false;
    }

    connection_bb_[ci.id] = drawConnection(painter, from.get(), to.get(), ci.id);
}

std::vector<QRectF> DesignerScene::drawConnection(QPainter *painter,
                                                  Connector *from, Connector *to,
                                                  int id)
{
    Port* from_port = getPort(from->getUUID());
    Port* to_port = getPort(to->getUUID());

    if(!from_port || !to_port) {
        return std::vector<QRectF>();
    }

    if(to->isAsynchronous()) {
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
    ccs.error = false;
    ccs.minimized_from = from_port->isMinimizedSize();
    ccs.minimized_to = to_port->isMinimizedSize();
    ccs.hidden_from = !from_port->isVisible();
    ccs.hidden_to = !to_port->isVisible();
    ccs.selected_from = from_port->property("focused").toBool();
    ccs.selected_to = to_port->property("focused").toBool();

    if(from->isAsynchronous()) {
        ccs.start_pos = BOTTOM;
    } else {
        ccs.start_pos = from_port->isFlipped() ? LEFT : RIGHT;
    }
    if(to->isAsynchronous()) {
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
        //        scale_factor *= 0.5;
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

    Fulcrum first(-1, convert(from), Fulcrum::FULCRUM_OUT, convert(from), convert(from));
    const Fulcrum* current = &first;
//    const Fulcrum* last = current;

    std::vector<Fulcrum> targets;
    if(id >= 0) {
        ConnectionDescription connection = graph_facade_->getConnectionWithId(id);
        targets = connection.fulcrums;
    }
    targets.emplace_back(-1, convert(to), Fulcrum::FULCRUM_IN, convert(to), convert(to));

    int sub_section = 0;

    QPointF cp1, cp2;

    // paths
    typedef std::pair<QPainterPath, int> Path;
    std::vector<Path> paths;

    // generate lines
    for(std::size_t i = 0; i < targets.size(); ++i) {
        const Fulcrum& next = targets[i];

        QPointF current_pos = convert(current->pos());
        QPointF next_pos = convert(next.pos());

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
            const Fulcrum& last = targets[sub_section-1];
            if(last.type() == Fulcrum::FULCRUM_LINEAR) {
                cp1 = current_pos;
            } else {
                cp1 = current_pos + convert(current->handleOut());
            }
        }

        if(next.type() == Fulcrum::FULCRUM_OUT) {
            cp2 = offset(next_pos, ccs.start_pos, x_offset) + y_offset;

        } else if(next.type() == Fulcrum::FULCRUM_IN) {
            cp2 = offset(next_pos, ccs.end_pos, x_offset) + y_offset;

        } else {
            if(next.type() == Fulcrum::FULCRUM_LINEAR) {
                cp2 = next_pos;
            } else {
                cp2 = next_pos + convert(next.handleIn());
            }
        }

        path.cubicTo(cp1, cp2, next_pos);

        paths.push_back(std::make_pair(path, sub_section));

//        last = current;
        current = &next;
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
        color_start.setAlpha(200);
        color_end.setAlpha(200);
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
    ConnectorPtr c = port->getAdaptee();
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

Port* DesignerScene::getPort(const UUID& connector_uuid)
{
    auto pos = port_map_.find(connector_uuid);
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

    ConnectorPtr c = p->getAdaptee();
    if(!c) {
        return;
    }

    bool is_message = c->isSynchronous();

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
        };

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
    ConnectionDescription c = graph_facade_->getConnectionWithId(highlight_connection_id_);

    QMenu menu;
    QAction* reset = new QAction("reset connection", &menu);
    menu.addAction(reset);
    QAction* del = new QAction("delete connection", &menu);
    menu.addAction(del);

    menu.addSeparator();

    QAction* active = new QAction("allow active tokens", &menu);
    active->setCheckable(true);
    active->setChecked(c.active);
    menu.addAction(active);

    QAction* selectedItem = menu.exec(QCursor::pos());

    if(selectedItem == del) {
        view_core_.getCommandDispatcher()->execute(CommandFactory(graph_facade_.get()).deleteConnectionByIdCommand(highlight_connection_id_));

    } else if(selectedItem == reset) {
        view_core_.getCommandDispatcher()->execute(CommandFactory(graph_facade_.get()).deleteAllConnectionFulcrumsCommand(highlight_connection_id_));

    } else if(selectedItem == active) {
        view_core_.getCommandDispatcher()->execute(CommandFactory(graph_facade_.get()).setConnectionActive(highlight_connection_id_, active->isChecked()));
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
            ConnectorPtr from = c.from.lock();
            if(from) {
                if(c.is_connected) {
                    ConnectorPtr to = c.to_c.lock();
                    if(to) {
                        ss << " - " << from->getUUID() << " => " << to->getUUID() << '\n';
                    }
                } else {
                    ss << " - " << from->getUUID() << " => [" << c.to_p.x() << ", " << c.to_p.y() << "]\n";
                }
            }
        }
    }
    return ss.str();
}

/// MOC
#include "../../../include/csapex/view/designer/moc_designer_scene.cpp"
