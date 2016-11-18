/// HEADER
#include <csapex/view/designer/graph_view.h>

/// COMPONENT
#include <csapex/command/add_node.h>
#include <csapex/command/command_factory.h>
#include <csapex/command/create_thread.h>
#include <csapex/command/delete_node.h>
#include <csapex/command/disable_node.h>
#include <csapex/command/dispatcher.h>
#include <csapex/command/flip_sides.h>
#include <csapex/command/group_nodes.h>
#include <csapex/command/ungroup_nodes.h>
#include <csapex/command/meta.h>
#include <csapex/command/playback_command.h>
#include <csapex/command/minimize.h>
#include <csapex/command/mute_node.h>
#include <csapex/command/move_box.h>
#include <csapex/command/rename_node.h>
#include <csapex/command/rename_connector.h>
#include <csapex/command/paste_graph.h>
#include <csapex/command/set_color.h>
#include <csapex/command/add_connection.h>
#include <csapex/command/add_variadic_connector.h>
#include <csapex/command/set_execution_mode.h>
#include <csapex/command/set_logger_level.h>
#include <csapex/core/graphio.h>
#include <csapex/core/csapex_core.h>
#include <csapex/core/settings.h>
#include <csapex/factory/node_factory.h>
#include <csapex/factory/snippet_factory.h>
#include <csapex/model/graph_facade.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/tag.h>
#include <csapex/model/graph/vertex.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/scheduling/thread_group.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/serialization/snippet.h>
#include <csapex/signal/slot.h>
#include <csapex/signal/event.h>
#include <csapex/view/designer/designer_scene.h>
#include <csapex/view/designer/drag_io.h>
#include <csapex/view/designer/graph_view_context_menu.h>
#include <csapex/view/node/box.h>
#include <csapex/view/node/note_box.h>
#include <csapex/view/node/default_node_adapter.h>
#include <csapex/view/node/node_adapter_factory.h>
#include <csapex/view/node/node_adapter.h>
#include <csapex/view/utility/clipboard.h>
#include <csapex/view/widgets/box_dialog.h>
#include <csapex/view/widgets/message_preview_widget.h>
#include <csapex/view/widgets/movable_graphics_proxy_widget.h>
#include <csapex/view/widgets/port.h>
#include <csapex/view/widgets/profiling_widget.h>
#include <csapex/view/widgets/port_panel.h>
#include <csapex/view/widgets/rewiring_dialog.h>
#include <csapex/csapex_mime.h>
#include <csapex/plugin/plugin_locator.h>

/// SYSTEM
#include <iostream>
#include <QKeyEvent>
#include <QApplication>
#include <QShortcut>
#include <QGraphicsItem>
#include <QInputDialog>
#include <QScrollBar>
#include <QDrag>
#include <QMimeData>
#include <QColorDialog>
#include <QSizeGrip>
#include <QFileDialog>
#include <QMessageBox>
#include <QRegularExpression>

using namespace csapex;

GraphView::GraphView(csapex::GraphFacadePtr graph_facade, CsApexViewCore& view_core, QWidget* parent)
    : QGraphicsView(parent),  core_(view_core.getCore()), view_core_(view_core),
      scene_(new DesignerScene(graph_facade, view_core)),
      graph_facade_(graph_facade),
      scalings_to_perform_(0), middle_mouse_dragging_(false), move_event_(nullptr),
      preview_widget_(nullptr)
{
    setCacheMode(QGraphicsView::CacheBackground);
    setViewportUpdateMode(QGraphicsView::FullViewportUpdate);

    setScene(scene_);
    setFocusPolicy(Qt::StrongFocus);
    setFocus(Qt::OtherFocusReason);

    setAcceptDrops(true);

    setDragMode(QGraphicsView::RubberBandDrag);
    setInteractive(true);

    QShortcut *box_reset_view = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_0), this);
    QObject::connect(box_reset_view, SIGNAL(activated()), this, SLOT(resetZoom()));

    QShortcut *box_zoom_in = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_Plus), this);
    QObject::connect(box_zoom_in, SIGNAL(activated()), this, SLOT(zoomIn()));

    QShortcut *box_zoom_out = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_Minus), this);
    QObject::connect(box_zoom_out, SIGNAL(activated()), this, SLOT(zoomOut()));

    QObject::connect(scene_, SIGNAL(selectionChanged()), this, SLOT(updateSelection()));
    QObject::connect(scene_, SIGNAL(selectionChanged()), this, SIGNAL(selectionChanged()));

    //    QObject::connect(&scalings_animation_timer_, SIGNAL(timeout()), this, SLOT(animateZoom()));
    QObject::connect(&scroll_animation_timer_, SIGNAL(timeout()), this, SLOT(animateScroll()));

    QObject::connect(this, SIGNAL(startProfilingRequest(NodeWorker*)), this, SLOT(startProfiling(NodeWorker*)));
    QObject::connect(this, SIGNAL(stopProfilingRequest(NodeWorker*)), this, SLOT(stopProfiling(NodeWorker*)));

    setContextMenuPolicy(Qt::DefaultContextMenu);


    QObject::connect(this, SIGNAL(triggerConnectorCreated(ConnectablePtr)), this, SLOT(connectorCreated(ConnectablePtr)));
    QObject::connect(this, SIGNAL(triggerConnectorRemoved(ConnectablePtr)), this, SLOT(connectorRemoved(ConnectablePtr)));

    qRegisterMetaType < ConnectablePtr > ("ConnectablePtr");

    scoped_connections_.emplace_back(graph_facade_->nodeWorkerAdded.connect([this](NodeWorkerPtr n) { nodeAdded(n); }));
    scoped_connections_.emplace_back(graph_facade_->nodeRemoved.connect([this](NodeHandlePtr n) { nodeRemoved(n); }));

    SubgraphNode* graph = graph_facade_->getSubgraphNode();

    scoped_connections_.emplace_back(graph->internalConnectionInProgress.connect([this](Connectable* from, Connectable* to) { scene_->previewConnection(from, to); }));
    scoped_connections_.emplace_back(graph->state_changed.connect([this](){ updateBoxInformation(); }));

    for(auto it = graph->beginVertices(); it != graph->endVertices(); ++it) {
        const NodeHandlePtr& nh = (*it)->getNodeHandle();
        apex_assert_hard(nh.get());
        NodeWorkerPtr nw = graph_facade_->getNodeWorker(nh.get());
        apex_assert_hard(nw);
        nodeAdded(nw);
    }

    setupWidgets();
}


GraphView::~GraphView()
{
    handle_connections_.clear();
    worker_connections_.clear();

    delete scene_;
}

void GraphView::setupWidgets()
{
    AUUID parent = getGraphFacade()->getAbsoluteUUID();

    outputs_widget_ = new PortPanel(ConnectorType::OUTPUT, scene_);
    QObject::connect(outputs_widget_, &PortPanel::portAdded, this, &GraphView::addPort);
    QObject::connect(outputs_widget_, &PortPanel::createPortRequest, this, &GraphView::createPort);
    QObject::connect(outputs_widget_, &PortPanel::createPortAndConnectRequest, this, &GraphView::createPortAndConnect);
    QObject::connect(outputs_widget_, &PortPanel::createPortAndMoveRequest, this, &GraphView::createPortAndMove);
    outputs_widget_proxy_ = scene_->addWidget(outputs_widget_);
    outputs_widget_->setup(graph_facade_);


    inputs_widget_ = new PortPanel(ConnectorType::INPUT, scene_);
    QObject::connect(inputs_widget_, &PortPanel::portAdded, this, &GraphView::addPort);
    QObject::connect(inputs_widget_, &PortPanel::createPortRequest, this, &GraphView::createPort);
    QObject::connect(inputs_widget_, &PortPanel::createPortAndConnectRequest, this, &GraphView::createPortAndConnect);
    QObject::connect(inputs_widget_, &PortPanel::createPortAndMoveRequest, this, &GraphView::createPortAndMove);
    inputs_widget_proxy_ = scene_->addWidget(inputs_widget_);
    inputs_widget_->setup(graph_facade_);


    slots_widget_ = new PortPanel(ConnectorType::SLOT_T, scene_);
    QObject::connect(slots_widget_, &PortPanel::portAdded, this, &GraphView::addPort);
    QObject::connect(slots_widget_, &PortPanel::createPortRequest, this, &GraphView::createPort);
    QObject::connect(slots_widget_, &PortPanel::createPortAndConnectRequest, this, &GraphView::createPortAndConnect);
    QObject::connect(slots_widget_, &PortPanel::createPortAndMoveRequest, this, &GraphView::createPortAndMove);
    slots_widget_proxy_ = scene_->addWidget(slots_widget_);
    slots_widget_->setup(graph_facade_);


    events_widget_ = new PortPanel(ConnectorType::EVENT, scene_);
    QObject::connect(events_widget_, &PortPanel::portAdded, this, &GraphView::addPort);
    QObject::connect(events_widget_, &PortPanel::createPortRequest, this, &GraphView::createPort);
    QObject::connect(events_widget_, &PortPanel::createPortAndConnectRequest, this, &GraphView::createPortAndConnect);
    QObject::connect(events_widget_, &PortPanel::createPortAndMoveRequest, this, &GraphView::createPortAndMove);
    triggers_widget_proxy_ = scene_->addWidget(events_widget_);
    events_widget_->setup(graph_facade_);

    // if this is a nested graph -> enable meta port
    if(graph_facade_->getParent()) {
        outputs_widget_->enableMetaPort(parent);
        inputs_widget_->enableMetaPort(parent);
        slots_widget_->enableMetaPort(parent);
        events_widget_->enableMetaPort(parent);
    } else {
        // hide panels in an empty top level graph
        if(scene_->isEmpty()) {
            outputs_widget_->hide();
            inputs_widget_->hide();
            slots_widget_->hide();
            events_widget_->hide();
        }
    }


    QObject::connect(scene_, &DesignerScene::changed, [this]() {
        bool visible = !scene_->isEmpty();

        // hide panels in an empty top level graph
        if(!graph_facade_->getParent()) {
            outputs_widget_->setVisible(visible);
            inputs_widget_->setVisible(visible);
            slots_widget_->setVisible(visible);
            events_widget_->setVisible(visible);
        }
    });
}

void GraphView::paintEvent(QPaintEvent *e)
{
    QPointF tl_view = mapToScene(QPoint(0, 0));
    QPointF br_view = mapToScene(QPoint(viewport()->width(), viewport()->height()));

    QPointF mid = 0.5 * (tl_view + br_view);

    {
        QPointF pos(tl_view.x(),
                    mid.y() - outputs_widget_->height() / 2.0);
        if(pos != outputs_widget_proxy_->pos()) {
            outputs_widget_proxy_->setPos(pos);
        }
    }
    {
        QPointF pos(br_view.x()-inputs_widget_->width(),
                    mid.y() - inputs_widget_->height() / 2.0);
        if(pos != inputs_widget_proxy_->pos()) {
            inputs_widget_proxy_->setPos(pos);
        }
    }
    {
        QPointF pos(mid.x() - slots_widget_->width() / 2.0,
                    br_view.y() - slots_widget_->height());
        if(pos != slots_widget_proxy_->pos()) {
            slots_widget_proxy_->setPos(pos);
        }
    }
    {
        QPointF pos(mid.x() - events_widget_->width() / 2.0,
                    tl_view.y());
        if(pos != triggers_widget_proxy_->pos()) {
            triggers_widget_proxy_->setPos(pos);
        }
    }
    QGraphicsView::paintEvent(e);

    Q_EMIT viewChanged();
}

void GraphView::drawForeground(QPainter *painter, const QRectF &rect)
{
    QGraphicsView::drawForeground(painter, rect);

    if(view_core_.isDebug()) {
        SubgraphNode* graph = graph_facade_->getSubgraphNode();

        QString debug_info =  QString::fromStdString(graph->makeStatusString());

        debug_info += QString::fromStdString(scene_->makeStatusString());

        QFont font("Ubuntu Condensed, Liberation Sans, FreeSans, Arial", 8, QFont::Normal);
        painter->setFont(font);

        painter->setPen(QPen(Qt::black));

        QFontMetrics fm(font);
        QRectF box = fm.boundingRect(rect.toRect(), Qt::AlignLeft, debug_info);

        box.translate(rect.width() - box.width() - 1, rect.height() - box.height() - 1);

        painter->fillRect(box, Qt::white);
        painter->drawText(box, debug_info);
    }
}

void GraphView::resizeEvent(QResizeEvent *event)
{
    scene_->setSceneRect(scene_->itemsBoundingRect());

    QGraphicsView::resizeEvent(event);
}

void GraphView::scrollContentsBy(int dx, int dy)
{
    QRectF min_r = scene_->itemsBoundingRect();

    QPointF tl_view = mapToScene(QPoint(0, 0));
    QPointF br_view = mapToScene(QPoint(width(), height()));

    double mx = std::abs(dx) + 10;
    double my = std::abs(dy) + 10;

    QPointF tl(std::min(tl_view.x() - mx, min_r.x()),
               std::min(tl_view.y() - my, min_r.y()));
    QPointF br(std::max(br_view.x() + mx, min_r.x() + min_r.width()),
               std::max(br_view.y() + my, min_r.y() + min_r.height()));

    QRectF expanded(tl, br);

    if(expanded != sceneRect()) {
        scene_->setSceneRect(expanded);
    }

    QGraphicsView::scrollContentsBy(dx, dy);
}

void GraphView::centerOnPoint(QPointF point)
{
    centerOn(point);
}


void GraphView::reset()
{
    scene_->clear();
    boxes_.clear();
    selected_boxes_.clear();
    profiling_.clear();
    profiling_connections_.clear();

    box_map_.clear();
    proxy_map_.clear();

    setupWidgets();

    update();
}

void GraphView::resetZoom()
{
    resetTransform();
    scene_->setScale(1.0);
}

void GraphView::zoomIn()
{
    zoom(5.0);
}

void GraphView::zoomOut()
{
    zoom(-5.0);
}

void GraphView::zoomAt(QPointF point, double f)
{
    zoom(f);
    centerOn(point);
}

void GraphView::zoom(double f)
{
    qreal factor = 1.0 + f / 25.0;

    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

    scale(factor, factor);
    scene_->setScale(transform().m11());
    scene_->invalidateSchema();
}

void GraphView::animateZoom()
{
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

    double current_scale = transform().m11();
    if((current_scale < 0.1 && scalings_to_perform_ < 0) ||
            (current_scale > 3.0 && scalings_to_perform_ > 0)) {
        scalings_to_perform_ = 0;
    }

    qreal factor = 1.0 + scalings_to_perform_ / 500.0;
    scale(factor, factor);

    scene_->setScale(transform().m11());
    scene_->invalidateSchema();

    if(scalings_to_perform_ > 0) {
        --scalings_to_perform_;
    } else if(scalings_to_perform_ < 0) {
        ++scalings_to_perform_;
    } else {
        scalings_animation_timer_.stop();
    }
}

DesignerScene* GraphView::designerScene()
{
    return scene_;
}

std::vector<NodeBox*> GraphView::boxes()
{
    return boxes_;
}

std::vector<NodeBox*> GraphView::getSelectedBoxes() const
{
    return selected_boxes_;
}

std::vector<UUID> GraphView::getSelectedUUIDs() const
{
    std::vector<UUID> uuids;
    uuids.reserve(selected_boxes_.size());
    for(const auto& box : selected_boxes_) {
        uuids.push_back(box->getNodeWorker()->getUUID());
    }
    return uuids;
}

void GraphView::enableSelection(bool enabled)
{
    selected_boxes_ = scene_->getSelectedBoxes();

    command::Meta::Ptr cmd(new command::Meta(graph_facade_->getAbsoluteUUID(), enabled ? "enable nodes" : "disable nodes"));

    for(NodeBox* box: selected_boxes_) {
        cmd->add(std::make_shared<command::DisableNode>(graph_facade_->getAbsoluteUUID(), box->getNodeHandle()->getUUID(), !enabled));
    }

    view_core_.execute(cmd);
}

void GraphView::updateSelection()
{
    selected_boxes_ = scene_->getSelectedBoxes();

    QList<QGraphicsItem *> selected = scene_->items();
    for(QGraphicsItem* item : selected) {
        MovableGraphicsProxyWidget* m = dynamic_cast<MovableGraphicsProxyWidget*>(item);
        if(m) {
            NodeBox* box = m->getBox();
            if(box && box->isVisible()) {
                box->setSelected(m->isSelected());
            }
        }
    }
}

Command::Ptr GraphView::deleteSelected()
{
    CommandFactory factory(graph_facade_.get());

    std::vector<UUID> uuids;
    for(QGraphicsItem* item : scene_->selectedItems()) {
        MovableGraphicsProxyWidget* proxy = dynamic_cast<MovableGraphicsProxyWidget*>(item);
        if(proxy) {
            uuids.push_back(proxy->getBox()->getNodeHandle()->getUUID());
        }
    }

    return factory.deleteAllNodes(uuids);
}

void GraphView::keyPressEvent(QKeyEvent* e)
{
    QGraphicsView::keyPressEvent(e);

    if(e->key() == Qt::Key_Space && Qt::ControlModifier != QApplication::keyboardModifiers() && !e->isAutoRepeat()) {
        if(dragMode() != QGraphicsView::ScrollHandDrag) {
            setDragMode(QGraphicsView::ScrollHandDrag);
            setInteractive(false);
            e->accept();
        }
    }
}

void GraphView::keyReleaseEvent(QKeyEvent* e)
{
    QGraphicsView::keyReleaseEvent(e);

    if(e->key() == Qt::Key_Space && !e->isAutoRepeat()) {
        setDragMode(QGraphicsView::RubberBandDrag);
        setInteractive(true);
        e->accept();
    }
}

void GraphView::mousePressEvent(QMouseEvent* e)
{
    bool was_rubber_band = false;
    if(e->button() == Qt::MiddleButton) {
        if(dragMode() == QGraphicsView::RubberBandDrag) {
            was_rubber_band = true;
            setDragMode(QGraphicsView::NoDrag);
        }
    }
    QGraphicsView::mousePressEvent(e);

    if(e->button() == Qt::MiddleButton && !e->isAccepted() && !middle_mouse_dragging_) {
        middle_mouse_dragging_ = true;
        middle_mouse_panning_ = false;
        middle_mouse_drag_start_ = e->screenPos();

    } else if(was_rubber_band) {
        setDragMode(QGraphicsView::RubberBandDrag);
    }
}

void GraphView::mouseReleaseEvent(QMouseEvent* e)
{
    QGraphicsView::mouseReleaseEvent(e);

    if(middle_mouse_dragging_) {
        middle_mouse_dragging_ = false;
        middle_mouse_panning_ = false;

        QMouseEvent fake(e->type(), e->pos(), Qt::LeftButton, Qt::LeftButton, e->modifiers());
        QGraphicsView::mouseReleaseEvent(&fake);

        setDragMode(QGraphicsView::RubberBandDrag);
        setInteractive(true);
        e->accept();
    }
}

void GraphView::mouseMoveEvent(QMouseEvent *e)
{
    if(middle_mouse_dragging_ && !middle_mouse_panning_) {
        auto delta = e->screenPos() - middle_mouse_drag_start_;
        if(std::hypot(delta.x(), delta.y()) > 10) {
            middle_mouse_panning_ = true;

            setDragMode(QGraphicsView::ScrollHandDrag);
            setInteractive(false);
            e->accept();

            QMouseEvent fake(e->type(), e->pos(), Qt::LeftButton, Qt::LeftButton, e->modifiers());
            QGraphicsView::mousePressEvent(&fake);
        }
    }

    QGraphicsView::mouseMoveEvent(e);

    setFocus();
}

void GraphView::wheelEvent(QWheelEvent *we)
{
    bool ctrl = Qt::ControlModifier & QApplication::keyboardModifiers();

    if(ctrl) {
        if(scene_->isEmpty()) {
            resetZoom();
            return;
        }

        we->accept();

        int scaleFactor = 4;
        int direction = (we->delta() > 0) ? 1 : -1;

        //        if((direction > 0) != (scalings_to_perform_ > 0)) {
        //            scalings_to_perform_ = 0;
        //        }

        //        scalings_to_perform_ +=  2 * direction * scaleFactor;

        zoom(direction * scaleFactor);

        //        if(!scalings_animation_timer_.isActive()) {
        //            scalings_animation_timer_.setInterval(1000.0 / 60.0);
        //            scalings_animation_timer_.start();
        //        }

    } else {
        QGraphicsView::wheelEvent(we);
    }
}

void GraphView::dragEnterEvent(QDragEnterEvent* e)
{
    delete move_event_;
    move_event_ = nullptr;

    if(!e->isAccepted()) {
        view_core_.getDragIO()->dragEnterEvent(this, e);

        QGraphicsView::dragEnterEvent(e);
    }
}

void GraphView::dragMoveEvent(QDragMoveEvent* e)
{
    QGraphicsItem* item_under_mouse = scene_->itemAt(mapToScene(e->pos()), QTransform());

    delete move_event_;
    move_event_ = new QDragMoveEvent(*e);

    QGraphicsView::dragMoveEvent(e);
    view_core_.getDragIO()->dragMoveEvent(this, e);

    if(item_under_mouse) {
        if(scroll_animation_timer_.isActive()) {
            scroll_animation_timer_.stop();
        }
    } else {
        static const int border_threshold = 20;
        static const double scroll_factor = 10.;

        bool scroll_p = false;

        QPointF pos = e->posF();
        if(pos.x() < border_threshold) {
            scroll_p = true;
            scroll_offset_x_ = scroll_factor * (pos.x() - border_threshold) / double(border_threshold);
        } else if(pos.x() > viewport()->width() - border_threshold) {
            scroll_p = true;
            scroll_offset_x_ = scroll_factor * (pos.x() - (viewport()->width() - border_threshold)) / double(border_threshold);
        } else {
            scroll_offset_x_ = 0;
        }

        if(pos.y() < border_threshold) {
            scroll_p = true;
            scroll_offset_y_ = scroll_factor * (pos.y() - border_threshold) / double(border_threshold);
        } else if(pos.y() > viewport()->height() - border_threshold) {
            scroll_p = true;
            scroll_offset_y_ = scroll_factor * (pos.y() - (viewport()->height() - border_threshold)) / double(border_threshold);
        } else {
            scroll_offset_y_ = 0;
        }

        if(scroll_p) {
            if(!scroll_animation_timer_.isActive()) {
                scroll_animation_timer_.start(1000./60.);
            }
        } else {
            if(scroll_animation_timer_.isActive()) {
                scroll_animation_timer_.stop();
            }
        }
    }
}

void GraphView::dropEvent(QDropEvent* e)
{
    delete move_event_;
    move_event_ = nullptr;

    view_core_.getDragIO()->dropEvent(this, e, mapToScene(e->pos()));
    if(!e->isAccepted()) {
        QGraphicsView::dropEvent(e);
    }

    if(scroll_animation_timer_.isActive()) {
        scroll_animation_timer_.stop();
    }
}

void GraphView::dragLeaveEvent(QDragLeaveEvent* e)
{
    delete move_event_;
    move_event_ = nullptr;

    QGraphicsView::dragLeaveEvent(e);

    if(scroll_animation_timer_.isActive()) {
        scroll_animation_timer_.stop();
    }
}

void GraphView::animateScroll()
{
    QScrollBar* h = horizontalScrollBar();
    h->setValue(h->value() + scroll_offset_x_);
    QScrollBar* v = verticalScrollBar();
    v->setValue(v->value() + scroll_offset_y_);

    if(move_event_) {
        view_core_.getDragIO()->dragMoveEvent(this, move_event_);
    }
}

void GraphView::showNodeInsertDialog()
{
    //    auto window =  QApplication::activeWindow();
    BoxDialog diag("Please enter the type of node to add.",
                   core_.getNodeFactory(), *view_core_.getNodeAdapterFactory(),
                   core_.getSnippetFactory());

    int r = diag.exec();

    if(r) {
        std::string mime = diag.getMIME();

        if(!mime.empty()) {
            createNodes(QCursor::pos(), diag.getName(), mime);
        }
    }
}


void GraphView::startPlacingBox(const std::string &type, NodeStatePtr state, const QPoint &offset)
{
    NodeConstructorPtr c = core_.getNodeFactory().getConstructor(type);
    NodeHandlePtr handle = c->makePrototype();

    if(!state) {
        state = handle->getNodeState();
    }

    apex_assert_hard(handle);

    QDrag* drag = new QDrag(this);
    QMimeData* mimeData = new QMimeData;

    mimeData->setData(QString::fromStdString(csapex::mime::node), type.c_str());
    if(state) {
        QVariant payload = qVariantFromValue((void *) &state);
        mimeData->setProperty("state", payload);
    }
    mimeData->setProperty("ox", offset.x());
    mimeData->setProperty("oy", offset.y());
    drag->setMimeData(mimeData);

    NodeBox* box = nullptr;

    bool is_note = type == "csapex::Note";

    if(is_note) {
        box = new NoteBox(core_.getSettings(), handle,
                          QIcon(QString::fromStdString(c->getIcon())));

    } else {
        box = new NodeBox(core_.getSettings(), handle,
                          QIcon(QString::fromStdString(c->getIcon())));
    }
    box->setAdapter(std::make_shared<DefaultNodeAdapter>(handle, box));

    if(state) {
        handle->setNodeState(state);
    }
    box->setStyleSheet(styleSheet());
    box->construct();
    box->setObjectName(handle->getType().c_str());

    if(!is_note) {
        box->setLabel(type);
    }

    drag->setPixmap(box->grab());
    drag->setHotSpot(-offset);
    drag->exec();

    delete box;
}


void GraphView::nodeAdded(NodeWorkerPtr node_worker)
{
    NodeHandlePtr node_handle = node_worker->getNodeHandle();
    std::string type = node_handle->getType();

    QIcon icon = QIcon(QString::fromStdString(core_.getNodeFactory().getConstructor(type)->getIcon()));

    NodeBox* box = nullptr;
    if(type == "csapex::Note") {
        box = new NoteBox(core_.getSettings(), node_handle, node_worker, icon, this);
    } else {
        box = new NodeBox(core_.getSettings(), node_handle, node_worker, icon, this);
    }


    QObject::connect(box, &NodeBox::createPortRequest, this, &GraphView::createPort);
    QObject::connect(box, &NodeBox::createPortAndConnectRequest, this, &GraphView::createPortAndConnect);
    QObject::connect(box, &NodeBox::createPortAndMoveRequest, this, &GraphView::createPortAndMove);

    //    box->executeCommand.connect(delegate::Delegate<void(CommandPtr)>(dispatcher_, &CommandDispatcher::execute));

    QObject::connect(box, &NodeBox::portAdded, this, &GraphView::addPort);
    QObject::connect(box, &NodeBox::portRemoved, this, &GraphView::removePort);

    NodeAdapter::Ptr adapter = view_core_.getNodeAdapterFactory()->makeNodeAdapter(node_handle, box);
    adapter->executeCommand.connect(delegate::Delegate<void(CommandPtr)>(&view_core_.getCommandDispatcher(), &CommandDispatcher::execute));
    box->setAdapter(adapter);

    box_map_[node_handle->getUUID()] = box;
    proxy_map_[node_handle->getUUID()] = new MovableGraphicsProxyWidget(box, this, view_core_);

    box->setStyleSheet(styleSheet());

    box->construct();

    addBox(box);

    // add existing connectors
    for(auto input : node_handle->getExternalInputs()) {
        connectorMessageAdded(input);
    }
    for(auto output : node_handle->getExternalOutputs()) {
        connectorMessageAdded(output);
    }
    for(auto slot : node_handle->getExternalSlots()) {
        connectorSignalAdded(slot);
    }
    for(auto event : node_handle->getExternalEvents()) {
        connectorSignalAdded(event);
    }

    // subscribe to coming connectors
    auto c1 = node_handle->connectorCreated.connect([this](ConnectablePtr c) { triggerConnectorCreated(c); });
    handle_connections_[node_handle.get()].emplace_back(c1);
    auto c2 = node_handle->connectorRemoved.connect([this](ConnectablePtr c) { triggerConnectorRemoved(c); });
    handle_connections_[node_handle.get()].emplace_back(c2);


    UUID uuid = node_handle->getUUID();
    QObject::connect(box, &NodeBox::toggled, [this, uuid](bool checked) {
        view_core_.execute(std::make_shared<command::DisableNode>(graph_facade_->getAbsoluteUUID(), uuid, !checked));
    });

    Q_EMIT boxAdded(box);
}


void GraphView::connectorCreated(ConnectablePtr connector)
{
    // TODO: dirty...
    if(dynamic_cast<Slot*> (connector.get()) || dynamic_cast<Event*>(connector.get())) {
        connectorSignalAdded(connector);
    } else {
        connectorMessageAdded(connector);
    }
}

void GraphView::connectorRemoved(ConnectablePtr connector)
{
    UUID parent_uuid = connector->getUUID().parentUUID();
    NodeBox* box = getBox(parent_uuid);

    box->removePort(connector);
}


void GraphView::connectorMessageAdded(ConnectablePtr connector)
{
    UUID parent_uuid = connector->getUUID().parentUUID();

    Graph* g = graph_facade_->getGraph();
    NodeHandle* node_worker = g->findNodeHandle(parent_uuid);
    if(node_worker) {
        Output* o = dynamic_cast<Output*>(connector.get());
        if(o && node_worker->isParameterOutput(o)) {
            return;
        }

        Input* i = dynamic_cast<Input*>(connector.get());
        if(i && node_worker->isParameterInput(i)) {
            return;
        }

        NodeBox* box = getBox(parent_uuid);
        QBoxLayout* layout = connector->isInput() ? box->getInputLayout() : box->getOutputLayout();

        box->createPort(connector, layout);
    }
}

void GraphView::connectorSignalAdded(ConnectablePtr connector)
{
    UUID parent_uuid = connector->getUUID().parentUUID();
    NodeBox* box = getBox(parent_uuid);
    if(box) {
        QBoxLayout* layout = dynamic_cast<Event*>(connector.get()) ? box->getEventLayout() : box->getSlotLayout();

        box->createPort(connector, layout);
    }
}

NodeBox* GraphView::getBox(const csapex::UUID &node_id)
{
    auto pos = box_map_.find(node_id);
    if(pos == box_map_.end()) {
        return nullptr;
    }

    return pos->second;
}

MovableGraphicsProxyWidget* GraphView::getProxy(const csapex::UUID &node_id)
{
    auto pos = proxy_map_.find(node_id);
    if(pos == proxy_map_.end()) {
        return nullptr;
    }

    return pos->second;
}

GraphFacade* GraphView::getGraphFacade() const
{
    return graph_facade_.get();
}

CsApexViewCore& GraphView::getViewCore() const
{
    return view_core_;
}

void GraphView::focusOnNode(const csapex::UUID &uuid)
{
    NodeBox* box = getBox(uuid);
    if(box) {
        scene_->setSelection(box);
        centerOn(box->graphicsProxyWidget());
    }
}

void GraphView::nodeRemoved(NodeHandlePtr node_handle)
{
    UUID node_uuid = node_handle->getUUID();
    NodeBox* box = getBox(node_uuid);
    box->stop();

    box_map_.erase(box_map_.find(node_uuid));
    proxy_map_.erase(proxy_map_.find(node_uuid));

    removeBox(box);

    box->deleteLater();

    Q_EMIT boxRemoved(box);
}

void GraphView::addBox(NodeBox *box)
{
    Graph* graph = graph_facade_->getGraph();

    QObject::connect(box, SIGNAL(renameRequest(NodeBox*)), this, SLOT(renameBox(NodeBox*)));

    NodeWorker* worker = box->getNodeWorker();
    NodeHandle* handle = worker->getNodeHandle().get();

    worker_connections_[worker].emplace_back(handle->connectionStart.connect([this](Connectable*) { scene_->deleteTemporaryConnections(); }));
    worker_connections_[worker].emplace_back(handle->connectionInProgress.connect([this](Connectable* from, Connectable* to) { scene_->previewConnection(from, to); }));
    worker_connections_[worker].emplace_back(handle->connectionDone.connect([this](Connectable*) { scene_->deleteTemporaryConnectionsAndRepaint(); }));


    QObject::connect(box, SIGNAL(showContextMenuForBox(NodeBox*,QPoint)), this, SLOT(showContextMenuForSelectedNodes(NodeBox*,QPoint)));

    worker_connections_[worker].emplace_back(worker->start_profiling.connect([this](NodeWorker* nw) { startProfilingRequest(nw); }));
    worker_connections_[worker].emplace_back(worker->stop_profiling.connect([this](NodeWorker* nw) { stopProfilingRequest(nw); }));


    MovableGraphicsProxyWidget* proxy = getProxy(box->getNodeWorker()->getUUID());
    scene_->addItem(proxy);

    QObject::connect(proxy, &MovableGraphicsProxyWidget::moved, this, &GraphView::movedBoxes);

    boxes_.push_back(box);

    for(QGraphicsItem *item : items()) {
        item->setFlag(QGraphicsItem::ItemIsMovable);
        item->setFlag(QGraphicsItem::ItemIsSelectable);
        item->setCacheMode(QGraphicsItem::DeviceCoordinateCache);
        item->setScale(1.0);
    }

    box->init();

    box->updateBoxInformation(graph);

    if(graph_facade_->getGraph()->countNodes() > 0) {
        setCacheMode(QGraphicsView::CacheNone);
        scene_->invalidate();
        setCacheMode(QGraphicsView::CacheBackground);
    }
}

void GraphView::removeBox(NodeBox *box)
{
    handle_connections_.erase(box->getNodeHandle());
    worker_connections_.erase(box->getNodeWorker());

    box->setVisible(false);
    box->deleteLater();

    auto pos = std::find(boxes_.begin(), boxes_.end(), box);
    if(pos != boxes_.end()) {
        boxes_.erase(pos);
    }
    profiling_.erase(box);

    if(graph_facade_->getGraph()->countNodes() == 0) {
        setCacheMode(QGraphicsView::CacheNone);
        scene_->invalidate();
        setCacheMode(QGraphicsView::CacheBackground);
    }
}


void GraphView::createPort(CreateConnectorRequest request)
{
    CommandFactory factory(graph_facade_.get());

    CommandPtr cmd = factory.createVariadicPort(request.target, request.connector_type, request.type, request.label, request.optional);
    view_core_.execute(cmd);
}

void GraphView::createPortAndConnect(CreateConnectorRequest request, Connectable* from)
{
    SubgraphNode* graph = graph_facade_->getSubgraphNode();
    AUUID graph_uuid = graph->getUUID().getAbsoluteUUID();

    CommandFactory factory(graph_facade_.get());

    std::shared_ptr<command::PlaybackCommand> playback = view_core_.getCommandDispatcher().make_playback(graph_uuid, "CreatePortAndConnect");

    if(request.target == graph->getUUID().getAbsoluteUUID()) {
        std::shared_ptr<command::AddVariadicConnector> add = std::make_shared<command::AddVariadicConnector>(graph_uuid, request.target, request.connector_type, request.type, request.label);
        playback->execute(add);

        RelayMapping ports = add->getMap();
        playback->execute(factory.addConnection(ports.internal, from->getUUID(), false));

    } else {
        std::shared_ptr<command::AddVariadicConnector> add = std::make_shared<command::AddVariadicConnector>(graph_uuid, request.target, request.connector_type, request.type, request.label);
        playback->execute(add);

        RelayMapping ports = add->getMap();
        playback->execute(factory.addConnection(ports.external, from->getUUID(), false));
    }

    view_core_.execute(playback);
}

void GraphView::createPortAndMove(CreateConnectorRequest request, Connectable* from)
{
    SubgraphNode* graph = graph_facade_->getSubgraphNode();
    AUUID graph_uuid = graph->getUUID().getAbsoluteUUID();

    CommandFactory factory(graph_facade_.get());

    std::shared_ptr<command::PlaybackCommand> playback = view_core_.getCommandDispatcher().make_playback(graph_uuid, "CreatePortAndMove");

    if(request.target == graph->getUUID().getAbsoluteUUID()) {
        std::shared_ptr<command::AddVariadicConnector> add = std::make_shared<command::AddVariadicConnector>(graph_uuid, request.target, request.connector_type, request.type, request.label);
        playback->execute(add);

        RelayMapping ports = add->getMap();
        playback->execute(factory.moveConnections(from->getUUID(), ports.internal));

    } else {
        std::shared_ptr<command::AddVariadicConnector> add = std::make_shared<command::AddVariadicConnector>(graph_uuid, request.target, request.connector_type, request.type, request.label);
        playback->execute(add);

        RelayMapping ports = add->getMap();
        playback->execute(factory.moveConnections(from->getUUID(), ports.external));
    }

    view_core_.execute(playback);
}

void GraphView::addPort(Port *port)
{
    scene_->addPort(port);

    QObject::connect(port, SIGNAL(mouseOver(Port*)), this, SLOT(showPreview(Port*)));
    QObject::connect(port, SIGNAL(mouseOut(Port*)), this, SLOT(stopPreview()));

    QObject::connect(port, &Port::removeConnectionsRequest, [this, port]() {
        ConnectablePtr adaptee = port->getAdaptee().lock();
        if(!adaptee) {
            return;
        }
        view_core_.execute(CommandFactory(graph_facade_.get()).removeAllConnectionsCmd(adaptee));
    });

    QObject::connect(port, &Port::addConnectionRequest, [this, port](Connectable* from) {
        ConnectablePtr adaptee = port->getAdaptee().lock();
        if(!adaptee) {
            return;
        }
        auto cmd = CommandFactory(graph_facade_.get()).addConnection(adaptee->getUUID(), from->getUUID(), false);
        view_core_.execute(cmd);
    });

    QObject::connect(port, &Port::moveConnectionRequest, [this, port](Connectable* from) {
        ConnectablePtr adaptee = port->getAdaptee().lock();
        if(!adaptee) {
            return;
        }
        Command::Ptr cmd = CommandFactory(graph_facade_.get()).moveConnections(from, adaptee.get());
        view_core_.execute(cmd);
    });

    QObject::connect(port, &Port::changePortRequest, [this, port](QString label) {
        ConnectablePtr adaptee = port->getAdaptee().lock();
        if(!adaptee) {
            return;
        }
        Command::Ptr cmd = std::make_shared<command::RenameConnector>(graph_facade_->getAbsoluteUUID(), adaptee->getUUID(), label.toStdString());
        view_core_.execute(cmd);
    });
}

void GraphView::removePort(Port *port)
{
    scene_->removePort(port);
}

void GraphView::renameBox(NodeBox *box)
{
    GraphFacade* graph = getGraphFacade();
    NodeHandle* node = box->getNodeHandle();
    NodeStatePtr state = node->getNodeState();
    QString old_name = QString::fromStdString(state->getLabel());

    bool ok = false;
    QString text = QInputDialog::getText(this, "Graph Label", "Enter new name",
                                         QLineEdit::Normal, old_name, &ok);
    if(ok) {
        if(old_name != text && !text.isEmpty()) {
            command::RenameNode::Ptr cmd(new command::RenameNode(graph->getAbsoluteUUID(),
                                                                 node->getUUID(),
                                                                 text.toStdString()));
            view_core_.execute(cmd);
        }
    }
}


void GraphView::startProfiling(NodeWorker *node)
{
    NodeBox* box = getBox(node->getUUID());
    apex_assert_hard(profiling_.find(box) == profiling_.end());

    ProfilingWidget* prof = new ProfilingWidget(box->getNodeWorker()->getProfiler(), node->getUUID().getFullName());
    profiling_[box] = prof;

    if(QVBoxLayout* vbl = dynamic_cast<QVBoxLayout*>(prof->layout())) {
        vbl->addWidget(new QSizeGrip(prof), 0, Qt::AlignBottom | Qt::AlignRight);
    }

    QObject::connect(box, &NodeBox::destroyed, prof, &ProfilingWidget::close);
    QObject::connect(box, &NodeBox::destroyed, prof, &ProfilingWidget::deleteLater);

    QGraphicsProxyWidget* prof_proxy = scene_->addWidget(prof);
    prof_proxy->setPos(box->graphicsProxyWidget()->pos() + QPointF(0,box->height()));
    prof->show();


    for (QGraphicsItem *item : items()) {
        item->setFlag(QGraphicsItem::ItemIsMovable);
        item->setFlag(QGraphicsItem::ItemIsSelectable);
        item->setCacheMode(QGraphicsItem::DeviceCoordinateCache);
        item->setScale(1.0);
    }

    MovableGraphicsProxyWidget* proxy = getProxy(box->getNodeWorker()->getUUID());
    QObject::connect(proxy, &MovableGraphicsProxyWidget::moving, [box, prof](double, double){
        QPointF pos = box->graphicsProxyWidget()->pos() + QPointF(0,box->height());
        prof->graphicsProxyWidget()->setPos(pos);
    });

    auto nw = box->getNodeWorker();

    auto cp = nw->messages_processed.connect([prof](){ prof->update(); });
    profiling_connections_[box].push_back(cp);

    auto ct = nw->ticked.connect([prof](){ prof->update(); });
    profiling_connections_[box].push_back(ct);
}

void GraphView::stopProfiling(NodeWorker *node)
{
    NodeBox* box = getBox(node->getUUID());

    for(auto& connection : profiling_connections_[box]) {
        connection.disconnect();
    }
    profiling_connections_.erase(box);

    std::map<NodeBox*, ProfilingWidget*>::iterator pos = profiling_.find(box);
    apex_assert_hard(pos != profiling_.end());

    pos->second->deleteLater();
    //    delete pos->second;
    profiling_.erase(pos);
}

void GraphView::movedBoxes(double dx, double dy)
{
    QPointF delta(dx, dy);
    command::Meta::Ptr meta(new command::Meta(graph_facade_->getAbsoluteUUID(), "move boxes"));
    for(QGraphicsItem* item : scene_->selectedItems()) {
        MovableGraphicsProxyWidget* proxy = dynamic_cast<MovableGraphicsProxyWidget*>(item);
        if(proxy) {
            NodeBox* b = proxy->getBox();
            QPointF to = proxy->pos();
            QPointF from = to - delta;
            meta->add(Command::Ptr(new command::MoveBox(graph_facade_->getAbsoluteUUID(),
                                                        b->getNodeWorker()->getUUID(),
                                                        Point(from.x(), from.y()), Point(to.x(), to.y()))));
        }
    }
    view_core_.execute(meta);

    scene_->invalidateSchema();
}

void GraphView::overwriteStyleSheet(const QString &stylesheet)
{
    setStyleSheet(stylesheet);

    scene_->update();

    for (NodeBox *box : boxes_) {
        box->setStyleSheet(stylesheet);
        box->updateVisuals();
    }

    outputs_widget_->setStyleSheet(stylesheet);
    inputs_widget_->setStyleSheet(stylesheet);
    slots_widget_->setStyleSheet(stylesheet);
    events_widget_->setStyleSheet(stylesheet);
}

void GraphView::updateBoxInformation()
{
    for(QGraphicsItem* item : scene_->items()) {
        MovableGraphicsProxyWidget* proxy = dynamic_cast<MovableGraphicsProxyWidget*>(item);
        if(proxy) {
            NodeBox* b = proxy->getBox();
            b->updateBoxInformation(graph_facade_->getGraph());
        }
    }
}

void GraphView::createNodes(const QPoint& global_pos, const std::string& type, const std::string& mime)
{
    if(mime == csapex::mime::node) {
        QPointF pos = mapToScene(mapFromGlobal(global_pos));

        AUUID graph_id = graph_facade_->getAbsoluteUUID();

        UUID uuid = graph_facade_->getGraph()->generateUUID(type);
        CommandPtr cmd(new command::AddNode(graph_id, type, Point (pos.x(), pos.y()), uuid, NodeStatePtr()));

        view_core_.execute(cmd);

    } else if(mime == csapex::mime::snippet) {
        QPointF pos = mapToScene(mapFromGlobal(global_pos));

        AUUID graph_id = graph_facade_->getAbsoluteUUID();
        CommandPtr cmd(new command::PasteGraph(graph_id, *core_.getSnippetFactory().getSnippet(type), Point (pos.x(), pos.y())));

        view_core_.execute(cmd);
    }
}

void GraphView::showContextMenuGlobal(const QPoint& global_pos)
{
    GraphViewContextMenu menu(*this);
    menu.showGlobalMenu(global_pos);
}

void GraphView::showContextMenuForSelectedNodes(NodeBox* box, const QPoint &scene_pos)
{
    if(std::find(selected_boxes_.begin(), selected_boxes_.end(), box) == selected_boxes_.end()) {
        scene_->setSelection(box);
        updateSelection();

    } else if(selected_boxes_.empty()) {
        selected_boxes_.push_back(box);
    }


    GraphViewContextMenu menu(*this);
    menu.showSelectionMenu(mapToGlobal(mapFromScene(scene_pos)));
}


void GraphView::usePrivateThreadFor()
{
    view_core_.execute(CommandFactory(graph_facade_.get()).switchThreadRecursively(getSelectedUUIDs(), ThreadGroup::PRIVATE_THREAD));
}


void GraphView::useDefaultThreadFor()
{
    view_core_.execute(CommandFactory(graph_facade_.get()).switchThreadRecursively(getSelectedUUIDs(), ThreadGroup::DEFAULT_GROUP_ID));
}

void GraphView::switchToThread(int group_id)
{
    view_core_.execute(CommandFactory(graph_facade_.get()).switchThreadRecursively(getSelectedUUIDs(), group_id));
}

void GraphView::createNewThreadGroupFor()
{
    bool ok;
    ThreadPool* thread_pool = graph_facade_->getThreadPool();
    QString text = QInputDialog::getText(this, "Group Name", "Enter new name", QLineEdit::Normal, QString::fromStdString(thread_pool->nextName()), &ok);

    if(ok && !text.isEmpty()) {
        command::Meta::Ptr cmd(new command::Meta(graph_facade_->getAbsoluteUUID(),"create new thread group"));
        for(NodeBox* box : selected_boxes_) {
            cmd->add(Command::Ptr(new command::CreateThread(graph_facade_->getAbsoluteUUID(),box->getNodeWorker()->getUUID(), text.toStdString())));
        }
        view_core_.execute(cmd);
    }
}

void GraphView::showProfiling(bool show)
{
    for(NodeBox* box : selected_boxes_) {
        box->showProfiling(show);
    }
}

void GraphView::chooseColor()
{
    QColor c = QColorDialog::getColor();

    if(!c.isValid()) {
        return;
    }

    int r = c.red();
    int g = c.green();
    int b = c.blue();

    command::Meta::Ptr cmd(new command::Meta(graph_facade_->getAbsoluteUUID(),"flip boxes"));
    for(NodeBox* box : selected_boxes_) {
        cmd->add(Command::Ptr(new command::SetColor(graph_facade_->getAbsoluteUUID(),box->getNodeWorker()->getUUID(), r, g, b)));
    }
    view_core_.execute(cmd);
}

void GraphView::flipBox()
{
    command::Meta::Ptr cmd(new command::Meta(graph_facade_->getAbsoluteUUID(),"flip boxes"));
    for(NodeBox* box : selected_boxes_) {
        cmd->add(Command::Ptr(new command::FlipSides(graph_facade_->getAbsoluteUUID(),box->getNodeWorker()->getUUID())));
    }
    view_core_.execute(cmd);
}

void GraphView::setExecutionMode(ExecutionMode mode)
{
    command::Meta::Ptr cmd(new command::Meta(graph_facade_->getAbsoluteUUID(),"set execution mode"));
    for(NodeBox* box : selected_boxes_) {
        cmd->add(Command::Ptr(new command::SetExecutionMode(graph_facade_->getAbsoluteUUID(),box->getNodeWorker()->getUUID(), mode)));
    }
    view_core_.execute(cmd);
}

void GraphView::setLoggerLevel(int level)
{
    command::Meta::Ptr cmd(new command::Meta(graph_facade_->getAbsoluteUUID(),"set logger level"));
    for(NodeBox* box : selected_boxes_) {
        cmd->add(Command::Ptr(new command::SetLoggerLevel(graph_facade_->getAbsoluteUUID(),box->getNodeWorker()->getUUID(), level)));
    }
    view_core_.execute(cmd);
}

void GraphView::minimizeBox(bool muted)
{
    command::Meta::Ptr cmd(new command::Meta(graph_facade_->getAbsoluteUUID(),(muted ? std::string("minimize") : std::string("maximize")) + " boxes"));
    for(NodeBox* box : selected_boxes_) {
        cmd->add(Command::Ptr(new command::Minimize(graph_facade_->getAbsoluteUUID(),box->getNodeWorker()->getUUID(), muted)));
    }
    view_core_.execute(cmd);
}

void GraphView::muteBox(bool mini)
{
    command::Meta::Ptr cmd(new command::Meta(graph_facade_->getAbsoluteUUID(),(mini ? std::string("mute") : std::string("unmute")) + " boxes"));
    for(NodeBox* box : selected_boxes_) {
        cmd->add(Command::Ptr(new command::MuteNode(graph_facade_->getAbsoluteUUID(),box->getNodeWorker()->getUUID(), mini)));
    }
    view_core_.execute(cmd);
}

void GraphView::deleteBox()
{
    command::Meta::Ptr cmd(new command::Meta(graph_facade_->getAbsoluteUUID(),"delete boxes", true));
    for(NodeBox* box : selected_boxes_) {
        cmd->add(Command::Ptr(new command::DeleteNode(graph_facade_->getAbsoluteUUID(),box->getNodeWorker()->getUUID())));
    }
    view_core_.execute(cmd);
}

void GraphView::morphNode()
{
    apex_assert_hard(getSelectedBoxes().size() == 1);

    const NodeBox* box = getSelectedBoxes()[0];

    NodeHandle* nh = box->getNodeHandle();

    RewiringDialog diag(nh, view_core_);
    diag.makeUI(styleSheet());

    int r = diag.exec();
    if(r) {
        std::string type = diag.getType();

        command::Meta::Ptr morph = std::make_shared<command::Meta>(graph_facade_->getAbsoluteUUID(), "change node type");

        CommandPtr delete_old = std::make_shared<command::DeleteNode>(graph_facade_->getAbsoluteUUID(), nh->getUUID());
        morph->add(delete_old);

        UUID new_uuid = graph_facade_->getGraph()->generateUUID(type);
        CommandPtr add_new = std::make_shared<command::AddNode>(graph_facade_->getAbsoluteUUID(),
                                                                type, nh->getNodeState()->getPos(), new_uuid, nullptr);
        morph->add(add_new);

        for(const ConnectionInformation& ci : diag.getConnections(new_uuid)) {
            morph->add(std::make_shared<command::AddConnection>(graph_facade_->getAbsoluteUUID(),
                                                                       ci.from, ci.to, ci.active));
        }

        view_core_.execute(morph);
    }
}

Snippet GraphView::serializeSelection() const
{
    GraphIO io(graph_facade_->getSubgraphNode(), &core_.getNodeFactory());

    std::vector<UUID> nodes;
    for(const NodeBox* box : selected_boxes_) {
        nodes.emplace_back(box->getNodeHandle()->getUUID());
    }

    return io.saveSelectedGraph(nodes);
}

void GraphView::copySelected()
{
    Snippet s = serializeSelection();
    YAML::Node yaml;
    s.toYAML(yaml);
    ClipBoard::set(yaml);
}

void GraphView::startCloningSelection(NodeBox* box_handle, const QPoint &offset)
{
    if(std::find(selected_boxes_.begin(), selected_boxes_.end(), box_handle) == selected_boxes_.end()) {
        box_handle->setSelected(true);
        selected_boxes_.push_back(box_handle);
    }

    Snippet snippet = serializeSelection();
    YAML::Node yaml;
    snippet.toYAML(yaml);
    std::stringstream yaml_txt;
    yaml_txt << yaml;


    Point box_pos = box_handle->getNodeHandle()->getNodeState()->getPos();
    Point top_left = box_pos;
    for(NodeBox* box : selected_boxes_) {
        Point pos = box->getNodeHandle()->getNodeState()->getPos();
        if(pos.x < top_left.x) {
            top_left.x = pos.x;
        }
        if(pos.y < top_left.y) {
            top_left.y = pos.y;
        }
    }

    QPoint insert_pos = offset;

    if(box_pos.x > top_left.x) {
        insert_pos.setX(insert_pos.x() + top_left.x - box_pos.x);
    }
    if(box_pos.y > top_left.y) {
        insert_pos.setY(insert_pos.y() + top_left.y - box_pos.y);
    }


    std::string type = box_handle->getNodeHandle()->getType();

    NodeConstructorPtr c = core_.getNodeFactory().getConstructor(type);
    NodeHandlePtr handle = c->makePrototype();

    apex_assert_hard(handle);

    QDrag* drag = new QDrag(this);
    QMimeData* mimeData = new QMimeData;

    auto data = QString::fromStdString(yaml_txt.str()).toUtf8();
    mimeData->setData("xcsapex/node-list", data);
    mimeData->setProperty("ox", insert_pos.x());
    mimeData->setProperty("oy", insert_pos.y());
    drag->setMimeData(mimeData);

    NodeBox* box = nullptr;

    bool is_note = type == "csapex::Note";

    if(is_note) {
        box = new NoteBox(core_.getSettings(), handle,
                          QIcon(QString::fromStdString(c->getIcon())));

    } else {
        box = new NodeBox(core_.getSettings(), handle,
                          QIcon(QString::fromStdString(c->getIcon())));
    }
    box->setAdapter(std::make_shared<DefaultNodeAdapter>(handle, box));

    handle->setNodeState(box_handle->getNodeHandle()->getNodeStateCopy());

    box->setStyleSheet(styleSheet());
    box->construct();
    box->setObjectName(handle->getType().c_str());

    if(!is_note) {
        box->setLabel(type);
    }

    drag->setPixmap(box->grab());
    drag->setHotSpot(-offset);
    drag->exec();

    delete box;
}


void GraphView::groupSelected()
{
    if(selected_boxes_.empty()) {
        return;
    }

    std::vector<UUID> uuids;
    uuids.reserve(selected_boxes_.size());
    for(NodeBox* box : selected_boxes_) {
        uuids.push_back(box->getNodeHandle()->getUUID());
    }
    CommandPtr cmd(new command::GroupNodes(graph_facade_->getAbsoluteUUID(),uuids));
    view_core_.execute(cmd);
}


void GraphView::ungroupSelected()
{
    apex_assert_hard(selected_boxes_.size() == 1);

    CommandPtr cmd(new command::UngroupNodes(graph_facade_->getAbsoluteUUID(), selected_boxes_.front()->getNodeHandle()->getUUID()));
    view_core_.execute(cmd);
}

void GraphView::makeSnippetFromSelected()
{
    if(selected_boxes_.empty()) {
        return;
    }

    NodeBox* first = selected_boxes_.at(0);
    QString default_name = QString::fromStdString(first->getLabel());

    QString name = QInputDialog::getText(QApplication::activeWindow(), "Name", "Please enter a descriptive name for the snippet.", QLineEdit::Normal, default_name);
    if(name.isEmpty()) {
        return;
    }

    QString description = QInputDialog::getMultiLineText(QApplication::activeWindow(), "Name", "Please enter a description for the snippet. (optional)");
    QString tags_string = QInputDialog::getText(QApplication::activeWindow(), "Tags", "Please enter tags for the snippet (comma separated).  (optional)", QLineEdit::Normal);

    std::vector<TagConstPtr> tags;
    for(const QString& qtag : tags_string.split(',')) {
        const std::string& stag = qtag.trimmed().toStdString();
        if(!stag.empty()){
            Tag::createIfNotExists(stag);
            tags.push_back(Tag::get(stag));
        }
    }

    std::vector<std::string> snippet_dirs = core_.getPluginLocator()->getPluginPaths("snippets");
    std::string first_snippet_dir = snippet_dirs.front();

    QString save_to_path;

    while(save_to_path.isEmpty()) {
        QString path = QFileDialog::getExistingDirectory(0, "Select directory for the snippet", QString::fromStdString(first_snippet_dir), QFileDialog::DontUseNativeDialog);
        if(path.isEmpty()) {
            return;
        }

        std::string path_s = path.toStdString();

        bool dir_indexed = false;
        for(const std::string& dir : snippet_dirs) {
            if(dir == path_s) {
                dir_indexed = true;
                break;
            }
        }

        if(!dir_indexed) {
            int r = QMessageBox::warning(this, tr("The directory is not indexed!"),
                                         QString("The directory ") + path + " is not indexed for snippets. Continue?",
                                         QMessageBox::Yes | QMessageBox::No);
            if(r == QMessageBox::Yes) {
                save_to_path = path;
            }
        } else {
            save_to_path = path;
        }
    }

    QDir directory(save_to_path);
    if(directory.exists()) {
        QString filename = name.toLower().replace(QRegularExpression("[\\/:?\"<>| ]"), "_");
        QFile file(save_to_path + QDir::separator() + filename + QString::fromStdString(Settings::template_extension));

        if(file.exists()) {
            int r = QMessageBox::warning(this, tr("Overwrite?"),
                                         QString("The file ") + file.fileName() + " already exist. Overwrite?",
                                         QMessageBox::Yes | QMessageBox::No);
            if(r == QMessageBox::No) {
                return;
            }
        }

        Snippet snippet = serializeSelection();

        snippet.setName(name.toStdString());
        snippet.setDescription(description.toStdString());
        snippet.setTags(tags);

        core_.getSnippetFactory().saveSnippet(snippet, file.fileName().toStdString());
    }
}

void GraphView::paste()
{
    apex_assert_hard(ClipBoard::canPaste());

    Snippet blueprint(ClipBoard::get());
    QPointF pos = mapToScene(mapFromGlobal(QCursor::pos()));

    AUUID graph_id = graph_facade_->getAbsoluteUUID();
    CommandPtr cmd(new command::PasteGraph(graph_id, blueprint, Point (pos.x(), pos.y())));

    view_core_.execute(cmd);
}

void GraphView::contextMenuEvent(QContextMenuEvent* event)
{
    if(scene_->getHighlightedConnectionId() != -1) {
        scene_->showConnectionContextMenu();
        return;
    }

    QGraphicsView::contextMenuEvent(event);

    if(!event->isAccepted()) {
        showContextMenuGlobal(event->globalPos());
    }
}

void GraphView::selectAll()
{
    for(QGraphicsItem* item : scene_->items()) {
        item->setSelected(true);
    }
}

void GraphView::showPreview(Port* port)
{
    QPointF pos = QCursor::pos() + QPointF(20, 20);

    if(!preview_widget_) {
        preview_widget_ = new MessagePreviewWidget;
        preview_widget_->hide();
    }

    preview_widget_->setWindowTitle(QString::fromStdString("Output"));
    preview_widget_->move(pos.toPoint());

    if(!preview_widget_->isConnected()) {
        preview_widget_->connectTo(port->getAdaptee().lock());
    }
}

void GraphView::stopPreview()
{
    if(preview_widget_) {
        preview_widget_->disconnect();
        preview_widget_->hide();
        preview_widget_->deleteLater();
        preview_widget_ = nullptr;
    }
}


void GraphView::useProfiler(std::shared_ptr<Profiler> profiler)
{
    Profilable::useProfiler(profiler);

    scene_->useProfiler(profiler);
}


/// MOC
#include "../../../include/csapex/view/designer/moc_graph_view.cpp"
