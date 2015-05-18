/// HEADER
#include <csapex/view/designer_view.h>

/// COMPONENT
#include <csapex/command/meta.h>
#include <csapex/command/move_box.h>
#include <csapex/command/delete_node.h>
#include <csapex/command/flip_sides.h>
#include <csapex/command/minimize.h>
#include <csapex/command/create_thread.h>
#include <csapex/command/switch_thread.h>
#include <csapex/view/box_dialog.h>
#include <csapex/model/node_factory.h>
#include <csapex/model/node.h>
#include <csapex/model/graph.h>
#include <csapex/command/dispatcher.h>
#include <csapex/command/add_node.h>
#include <csapex/view/designer_scene.h>
#include <csapex/view/box.h>
#include <csapex/utility/movable_graphics_proxy_widget.h>
#include <csapex/core/drag_io.h>
#include <csapex/view/widget_controller.h>
#include <csapex/view/profiling_widget.h>
#include <csapex/model/node_worker.h>
#include <csapex/utility/context_menu_handler.h>
#include <csapex/core/settings.h>
#include <csapex/core/thread_pool.h>
#include <csapex/view/box.h>

/// SYSTEM
#include <iostream>
#include <QKeyEvent>
#include <QApplication>
#include <QShortcut>
#include <QGraphicsItem>
#include <QGLWidget>
#include <QInputDialog>
#include <QScrollBar>

using namespace csapex;

DesignerView::DesignerView(DesignerScene *scene, csapex::GraphPtr graph,
                           Settings &settings, ThreadPool &thread_pool,
                           CommandDispatcher *dispatcher, WidgetControllerPtr widget_ctrl, DragIO& dragio, DesignerStyleable *style,
                           QWidget *parent)
    : QGraphicsView(parent), scene_(scene), style_(style), settings_(settings),
      thread_pool_(thread_pool), graph_(graph), dispatcher_(dispatcher), widget_ctrl_(widget_ctrl), drag_io_(dragio),
      scalings_to_perform_(0), move_event_(nullptr)

{
    //    setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers))); // memory leak?
    QGLFormat fmt;
    fmt.setSampleBuffers(true);
    fmt.setSamples(2);
    setViewport(new QGLWidget(fmt));

    setCacheMode(QGraphicsView::CacheBackground);
    setViewportUpdateMode(QGraphicsView::MinimalViewportUpdate);

    setScene(scene_);
    setFocusPolicy(Qt::StrongFocus);
    setFocus(Qt::OtherFocusReason);

    setAcceptDrops(true);

    setDragMode(QGraphicsView::RubberBandDrag);
    setInteractive(true);

    QShortcut *box_shortcut = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_Space), this);
    QObject::connect(box_shortcut, SIGNAL(activated()), this, SLOT(showBoxDialog()));

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

    setContextMenuPolicy(Qt::DefaultContextMenu);
}

DesignerView::~DesignerView()
{
    delete scene_;
}

void DesignerView::paintEvent(QPaintEvent *e)
{
    QGraphicsView::paintEvent(e);

    Q_EMIT viewChanged();
}

void DesignerView::centerOnPoint(QPointF point)
{
    centerOn(point);
    //    QScrollBar* h = horizontalScrollBar();
    //    QScrollBar* v = verticalScrollBar();

    //    int x = point.x();
    //    int y = point.y();

    //    h->setValue(x);
    //    v->setValue(y);
}

void DesignerView::reset()
{
    scene_->clear();
}

void DesignerView::resetZoom()
{
    resetTransform();
    scene_->setScale(1.0);
}

void DesignerView::zoomIn()
{
    zoom(5.0);
}

void DesignerView::zoomOut()
{
    zoom(-5.0);
}

void DesignerView::zoomAt(QPointF point, double f)
{
    zoom(f);
    centerOn(point);
}

void DesignerView::zoom(double f)
{
    qreal factor = 1.0 + f / 25.0;

    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

    scale(factor, factor);
    scene_->setScale(transform().m11());
    scene_->invalidateSchema();
}

void DesignerView::animateZoom()
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

DesignerScene* DesignerView::designerScene()
{
    return scene_;
}

std::vector<NodeBox*> DesignerView::boxes()
{
    return boxes_;
}

void DesignerView::updateSelection()
{
    QList<QGraphicsItem *> selected = scene_->items();
    foreach(QGraphicsItem* item, selected) {
        MovableGraphicsProxyWidget* m = dynamic_cast<MovableGraphicsProxyWidget*>(item);
        if(m) {
            NodeBox* box = m->getBox();
            if(box && box->isVisible()) {
                box->setSelected(m->isSelected());
            }
        }
    }
}

Command::Ptr DesignerView::deleteSelected()
{
    command::Meta::Ptr meta(new command::Meta("delete selected boxes"));
    foreach(QGraphicsItem* item, scene_->selectedItems()) {
        MovableGraphicsProxyWidget* proxy = dynamic_cast<MovableGraphicsProxyWidget*>(item);
        if(proxy) {
            meta->add(Command::Ptr(new command::DeleteNode(proxy->getBox()->getNodeWorker()->getUUID())));
        }
    }
    return meta;
}

void DesignerView::keyPressEvent(QKeyEvent* e)
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

void DesignerView::keyReleaseEvent(QKeyEvent* e)
{
    QGraphicsView::keyReleaseEvent(e);

    if(e->key() == Qt::Key_Space && !e->isAutoRepeat()) {
        setDragMode(QGraphicsView::RubberBandDrag);
        setInteractive(true);
        e->accept();
    }
}

void DesignerView::wheelEvent(QWheelEvent *we)
{
    bool shift = Qt::ShiftModifier & QApplication::keyboardModifiers();
    bool ctrl = Qt::ControlModifier & QApplication::keyboardModifiers();

    if(shift || ctrl) {
        if(scene_->isEmpty()) {
            resetZoom();
            return;
        }

        we->accept();

        int scaleFactor = shift ? 1 : 4;
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

void DesignerView::mouseMoveEvent(QMouseEvent *me)
{
    QGraphicsView::mouseMoveEvent(me);

    setFocus();
}

void DesignerView::dragEnterEvent(QDragEnterEvent* e)
{
    delete move_event_;
    move_event_ = nullptr;

    QGraphicsView::dragEnterEvent(e);
    drag_io_.dragEnterEvent(this, e);
}

void DesignerView::dragMoveEvent(QDragMoveEvent* e)
{
    delete move_event_;
    move_event_ = new QDragMoveEvent(*e);

    QGraphicsView::dragMoveEvent(e);
    if(!e->isAccepted()) {
        drag_io_.dragMoveEvent(this, e);
    }

    static const int border_threshold = 100;
    static const double scroll_factor = 10.;

    bool scroll_p = false;

    QPointF pos = e->posF();
    if(pos.x() < border_threshold) {
        scroll_p = true;
        scroll_offset_x_ = scroll_factor * (pos.x() - border_threshold) / double(border_threshold);
    } else if(pos.x() > width() - border_threshold) {
        scroll_p = true;
        scroll_offset_x_ = scroll_factor * (pos.x() - (width() - border_threshold)) / double(border_threshold);
    }

    if(pos.y() < border_threshold) {
        scroll_p = true;
        scroll_offset_y_ = scroll_factor * (pos.y() - border_threshold) / double(border_threshold);
    } else if(pos.y() > height() - border_threshold) {
        scroll_p = true;
        scroll_offset_y_ = scroll_factor * (pos.y() - (height() - border_threshold)) / double(border_threshold);
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

void DesignerView::dropEvent(QDropEvent* e)
{
    delete move_event_;
    move_event_ = nullptr;

    QGraphicsView::dropEvent(e);
    drag_io_.dropEvent(this, e, mapToScene(e->pos()));

    if(scroll_animation_timer_.isActive()) {
        scroll_animation_timer_.stop();
    }
}

void DesignerView::dragLeaveEvent(QDragLeaveEvent* e)
{
    delete move_event_;
    move_event_ = nullptr;

    QGraphicsView::dragLeaveEvent(e);

    if(scroll_animation_timer_.isActive()) {
        scroll_animation_timer_.stop();
    }
}

void DesignerView::animateScroll()
{
    QScrollBar* h = horizontalScrollBar();
    h->setValue(h->value() + scroll_offset_x_);
    QScrollBar* v = verticalScrollBar();
    v->setValue(v->value() + scroll_offset_y_);

    if(move_event_) {
        drag_io_.dragMoveEvent(this, move_event_);
    }
}

void DesignerView::showBoxDialog()
{
    BoxDialog diag(widget_ctrl_.get());
    int r = diag.exec();

    if(r) {
        std::string type = diag.getName();

        if(!type.empty() && widget_ctrl_->getNodeFactory()->isValidType(type)) {
            UUID uuid = UUID::make(graph_->makeUUIDPrefix(type));
            QPointF pos = mapToScene(mapFromGlobal(QCursor::pos()));
            dispatcher_->executeLater(Command::Ptr(new command::AddNode(type, pos.toPoint(), UUID::NONE, uuid, nullptr)));
        }
    }
}

void DesignerView::addBoxEvent(NodeBox *box)
{
    QObject::connect(box, SIGNAL(renameRequest(NodeBox*)), this, SLOT(renameBox(NodeBox*)));

    QObject::connect(box, SIGNAL(moved(NodeBox*, int, int)), scene_, SLOT(boxMoved(NodeBox*)));

    NodeWorker* worker = box->getNodeWorker();
    QObject::connect(worker, SIGNAL(connectionStart(Connectable*)), scene_, SLOT(deleteTemporaryConnections()), Qt::QueuedConnection);
    QObject::connect(worker, SIGNAL(connectionInProgress(Connectable*,Connectable*)), scene_, SLOT(previewConnection(Connectable*,Connectable*)), Qt::QueuedConnection);
    QObject::connect(worker, SIGNAL(connectionDone(Connectable*)), scene_, SLOT(deleteTemporaryConnectionsAndRepaint()), Qt::QueuedConnection);

    graph_->structureChanged.connect([this](Graph*){ updateBoxInformation(); });

    QObject::connect(box, SIGNAL(showContextMenuForBox(NodeBox*,QPoint)), this, SLOT(showContextMenuForSelectedNodes(NodeBox*,QPoint)));

    QObject::connect(worker, SIGNAL(startProfiling(NodeWorker*)), this, SLOT(startProfiling(NodeWorker*)));
    QObject::connect(worker, SIGNAL(stopProfiling(NodeWorker*)), this, SLOT(stopProfiling(NodeWorker*)));

    MovableGraphicsProxyWidget* proxy = widget_ctrl_->getProxy(box->getNodeWorker()->getUUID());
    scene_->addItem(proxy);

    QObject::connect(proxy, SIGNAL(moved(double,double)), this, SLOT(movedBoxes(double,double)));

    boxes_.push_back(box);

    box->setStyleSheet(styleSheet());

    foreach (QGraphicsItem *item, items()) {
        item->setFlag(QGraphicsItem::ItemIsMovable);
        item->setFlag(QGraphicsItem::ItemIsSelectable);
        item->setCacheMode(QGraphicsItem::DeviceCoordinateCache);
        item->setScale(1.0);
    }

    box->init();
    box->triggerPlaced();
    box->show();

    box->updateBoxInformation(graph_.get());
}

void DesignerView::renameBox(NodeBox *box)
{
    bool ok;
    QString text = QInputDialog::getText(this, "Box Label", "Enter new name", QLineEdit::Normal, box->getLabel().c_str(), &ok);

    if(ok && !text.isEmpty()) {
        box->setLabel(text);
    }
}

void DesignerView::removeBoxEvent(NodeBox *box)
{
    box->setVisible(false);
    box->deleteLater();

    boxes_.erase(std::find(boxes_.begin(), boxes_.end(), box));
    profiling_.erase(box);
}

void DesignerView::startProfiling(NodeWorker *node)
{
    NodeBox* box = widget_ctrl_->getBox(node->getUUID());
    apex_assert_hard(profiling_.find(box) == profiling_.end());

    ProfilingWidget* prof = new ProfilingWidget(this, box);
    profiling_[box] = prof;

    QGraphicsProxyWidget* prof_proxy = scene_->addWidget(prof);
    prof->reposition(prof_proxy->pos().x(), prof_proxy->pos().y());
    prof->show();


    foreach (QGraphicsItem *item, items()) {
        item->setFlag(QGraphicsItem::ItemIsMovable);
        item->setFlag(QGraphicsItem::ItemIsSelectable);
        item->setCacheMode(QGraphicsItem::DeviceCoordinateCache);
        item->setScale(1.0);
    }

    MovableGraphicsProxyWidget* proxy = widget_ctrl_->getProxy(box->getNodeWorker()->getUUID());
    QObject::connect(proxy, SIGNAL(moving(double,double)), prof, SLOT(reposition(double,double)));
    QObject::connect(box->getNodeWorker(), SIGNAL(messageProcessed()), prof, SLOT(update()));
    QObject::connect(box->getNodeWorker(), SIGNAL(ticked()), prof, SLOT(update()));
}

void DesignerView::stopProfiling(NodeWorker *node)
{
    NodeBox* box = widget_ctrl_->getBox(node->getUUID());

    std::map<NodeBox*, ProfilingWidget*>::iterator pos = profiling_.find(box);
    apex_assert_hard(pos != profiling_.end());

    pos->second->deleteLater();
    //    delete pos->second;
    profiling_.erase(pos);
}

void DesignerView::movedBoxes(double dx, double dy)
{
    QPointF delta(dx, dy);

    command::Meta::Ptr meta(new command::Meta("move boxes"));
    foreach(QGraphicsItem* item, scene_->selectedItems()) {
        MovableGraphicsProxyWidget* proxy = dynamic_cast<MovableGraphicsProxyWidget*>(item);
        if(proxy) {
            NodeBox* b = proxy->getBox();
            QPointF to = proxy->pos();
            QPointF from = to - delta;
            meta->add(Command::Ptr(new command::MoveBox(b->getNodeWorker()->getUUID(), from, to, *widget_ctrl_)));
        }
    }

    dispatcher_->execute(meta);
}

void DesignerView::overwriteStyleSheet(QString &stylesheet)
{
    setStyleSheet(stylesheet);

    scene_->update();

    foreach (NodeBox *box, boxes_) {
        box->setStyleSheet(stylesheet);
    }
}

void DesignerView::updateBoxInformation()
{
    foreach(QGraphicsItem* item, scene_->items()) {
        MovableGraphicsProxyWidget* proxy = dynamic_cast<MovableGraphicsProxyWidget*>(item);
        if(proxy) {
            NodeBox* b = proxy->getBox();
            b->updateBoxInformation(graph_.get());
        }
    }
}

void DesignerView::showContextMenuGlobal(const QPoint& global_pos)
{
    /// BOXES
    showContextMenuAddNode(global_pos);
}

void DesignerView::showContextMenuForSelectedNodes(NodeBox* box, const QPoint &scene_pos)
{
    auto selected_boxes = scene_->getSelectedBoxes();

    if(std::find(selected_boxes.begin(), selected_boxes.end(), box) == selected_boxes.end()) {
        scene_->setSelection(box);
        selected_boxes = scene_->getSelectedBoxes();

    } else if(selected_boxes.empty()) {
        selected_boxes.push_back(box);
    }

    std::stringstream title;
    if(selected_boxes.size() == 1) {
        title << "Node: " << selected_boxes.front()->getNodeWorker()->getUUID().getShortName();
    } else {
        title << selected_boxes.size() << " selected nodes";
    }

    QMenu menu;
    std::map<QAction*, std::function<void()> > handler;

    ContextMenuHandler::addHeader(menu, title.str());

    bool has_minimized = false;
    bool has_maximized = false;
    for(NodeBox* box : selected_boxes) {
        bool m = box->isMinimizedSize();
        has_minimized |= m;
        has_maximized |= !m;
    }
    if(has_minimized) {
        QAction* max = new QAction("maximize", &menu);
        max->setIcon(QIcon(":/maximize.png"));
        max->setIconVisibleInMenu(true);
        handler[max] = std::bind(&DesignerView::minimizeBox, this, selected_boxes, false);
        menu.addAction(max);
    }
    if(has_maximized){
        QAction* min = new QAction("minimize", &menu);
        min->setIcon(QIcon(":/minimize.png"));
        min->setIconVisibleInMenu(true);
        handler[min] = std::bind(&DesignerView::minimizeBox, this, selected_boxes, true);
        menu.addAction(min);
    }

    QAction* flip = new QAction("flip sides", &menu);
    flip->setIcon(QIcon(":/flip.png"));
    flip->setIconVisibleInMenu(true);
    handler[flip] = std::bind(&DesignerView::flipBox, this, selected_boxes);
    menu.addAction(flip);

    menu.addSeparator();

    bool threading = !settings_.get("threadless", false);
    bool grouping = settings_.get("thread_grouping", false);
    QMenu thread_menu("thread grouping", &menu);
    thread_menu.setEnabled(threading && !grouping);
    menu.addMenu(&thread_menu);

    if(thread_menu.isEnabled()) {
        QAction* private_thread = new QAction("private thread", &menu);
        private_thread->setIcon(QIcon(":/thread_group_none.png"));
        private_thread->setIconVisibleInMenu(true);
        handler[private_thread] = std::bind(&DesignerView::usePrivateThreadFor, this,  selected_boxes);
        thread_menu.addAction(private_thread);

        thread_menu.addSeparator();

        QMenu* choose_group_menu = new QMenu("thread group", &menu);

        std::vector<ThreadPool::Group> thread_groups = thread_pool_.getCustomGroups();
        for(std::size_t i = 0; i < thread_groups.size(); ++i) {
            const ThreadPool::Group& group = thread_groups[i];

            std::stringstream ss;
            ss << "(" << group.id << ") " << group.name;
            QAction* switch_thread = new QAction(QString::fromStdString(ss.str()), &menu);
            switch_thread->setIcon(QIcon(":/thread_group.png"));
            switch_thread->setIconVisibleInMenu(true);
            handler[switch_thread] = std::bind(&DesignerView::switchToThread, this, selected_boxes, group.id);
            choose_group_menu->addAction(switch_thread);
        }

        choose_group_menu->addSeparator();

        QAction* new_group = new QAction("new thread group", &menu);
        new_group->setIcon(QIcon(":/thread_group_add.png"));
        new_group->setIconVisibleInMenu(true);
        //        handler[new_group] = std::bind(&ThreadPool::createNewThreadGroupFor, &thread_pool_,  box->getNodeWorker());
        handler[new_group] = std::bind(&DesignerView::createNewThreadGroupFor, this,  selected_boxes);

        choose_group_menu->addAction(new_group);

        thread_menu.addMenu(choose_group_menu);
    }

    //    QAction* term = new QAction("terminate thread", &menu);
    //    term->setIcon(QIcon(":/stop.png"));
    //    term->setIconVisibleInMenu(true);
    //    handler[term] = std::bind(&NodeBox::killContent, box);
    //    menu.addAction(term);

    menu.addSeparator();


    bool has_profiling = false;
    bool has_not_profiling = false;
    for(NodeBox* box : selected_boxes) {
        bool p = box->getNodeWorker()->isProfiling();
        has_profiling |= p;
        has_not_profiling |= !p;
    }
    if(has_profiling) {
        QAction* prof = new QAction("stop profiling", &menu);
        prof->setIcon(QIcon(":/stop_profiling.png"));
        prof->setIconVisibleInMenu(true);
        handler[prof] = std::bind(&DesignerView::showProfiling, this, selected_boxes, false);
        menu.addAction(prof);
    }
    if(has_not_profiling){
        QAction* prof = new QAction("start profiling", &menu);
        prof->setIcon(QIcon(":/profiling.png"));
        prof->setIconVisibleInMenu(true);
        handler[prof] = std::bind(&DesignerView::showProfiling, this, selected_boxes, true);
        menu.addAction(prof);
    }

    if(selected_boxes.size() == 1) {
        QAction* info = new QAction("get information", &menu);
        info->setIcon(QIcon(":/help.png"));
        info->setIconVisibleInMenu(true);
        handler[info] = std::bind(&NodeBox::getInformation, selected_boxes.front());
        menu.addAction(info);
    }

    menu.addSeparator();

    QAction* del = new QAction("delete", &menu);
    del->setIcon(QIcon(":/close.png"));
    del->setIconVisibleInMenu(true);
    handler[del] = std::bind(&DesignerView::deleteBox, this, selected_boxes);
    menu.addAction(del);

    QAction* selectedItem = menu.exec(mapToGlobal(mapFromScene(scene_pos)));

    if(selectedItem) {
        handler[selectedItem]();
    }
}


void DesignerView::usePrivateThreadFor(const std::vector<NodeBox *>& boxes)
{
    command::Meta::Ptr cmd(new command::Meta("use private thread"));
    for(NodeBox* box : boxes) {
        cmd->add(Command::Ptr(new command::SwitchThread(box->getNodeWorker()->getUUID(), 0)));
    }
    dispatcher_->execute(cmd);
}

void DesignerView::switchToThread(const std::vector<NodeBox *>& boxes, int group_id)
{
    command::Meta::Ptr cmd(new command::Meta("switch thread"));
    for(NodeBox* box : boxes) {
        cmd->add(Command::Ptr(new command::SwitchThread(box->getNodeWorker()->getUUID(), group_id)));
    }
    dispatcher_->execute(cmd);
}

void DesignerView::createNewThreadGroupFor(const std::vector<NodeBox *>&boxes)
{
    bool ok;
    QString text = QInputDialog::getText(this, "Group Name", "Enter new name", QLineEdit::Normal, QString::fromStdString(thread_pool_.nextName()), &ok);

    if(ok && !text.isEmpty()) {
        command::Meta::Ptr cmd(new command::Meta("create new thread group"));
        for(NodeBox* box : boxes) {
            cmd->add(Command::Ptr(new command::CreateThread(box->getNodeWorker()->getUUID(), text.toStdString())));
        }
        dispatcher_->execute(cmd);
    }
}

void DesignerView::showProfiling(const std::vector<NodeBox *> &boxes, bool show)
{
    for(NodeBox* box : boxes) {
        box->showProfiling(show);
    }
}

void DesignerView::flipBox(const std::vector<NodeBox *>& boxes)
{
    command::Meta::Ptr cmd(new command::Meta("flip boxes"));
    for(NodeBox* box : boxes) {
        cmd->add(Command::Ptr(new command::FlipSides(box->getNodeWorker()->getUUID())));
    }
    dispatcher_->execute(cmd);
}

void DesignerView::minimizeBox(const std::vector<NodeBox *>& boxes, bool mini)
{
    command::Meta::Ptr cmd(new command::Meta((mini ? std::string("minimize") : std::string("maximize")) + " boxes"));
    for(NodeBox* box : boxes) {
        cmd->add(Command::Ptr(new command::Minimize(box->getNodeWorker()->getUUID(), mini)));
    }
    dispatcher_->execute(cmd);
}

void DesignerView::deleteBox(const std::vector<NodeBox *>& boxes)
{
    command::Meta::Ptr cmd(new command::Meta("delete boxes"));
    for(NodeBox* box : boxes) {
        cmd->add(Command::Ptr(new command::DeleteNode(box->getNodeWorker()->getUUID())));
    }
    dispatcher_->execute(cmd);
}

void DesignerView::contextMenuEvent(QContextMenuEvent* event)
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

void DesignerView::showContextMenuAddNode(const QPoint &global_pos)
{
    QMenu menu;
    widget_ctrl_->insertAvailableNodeTypes(&menu);

    QAction* selectedItem = menu.exec(global_pos);

    if(selectedItem) {
        std::string selected = selectedItem->data().toString().toStdString();
        widget_ctrl_->startPlacingBox(this, selected, nullptr);
    }
}

void DesignerView::selectAll()
{
    foreach(QGraphicsItem* item, scene_->items()) {
        item->setSelected(true);
    }
}
/// MOC
#include "../../include/csapex/view/moc_designer_view.cpp"
