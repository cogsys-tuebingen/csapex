/// HEADER
#include <csapex/view/designer_view.h>

/// COMPONENT
#include <csapex/command/meta.h>
#include <csapex/command/move_box.h>
#include <csapex/command/delete_node.h>
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

/// SYSTEM
#include <iostream>
#include <QKeyEvent>
#include <QApplication>
#include <QShortcut>
#include <QGraphicsItem>
#include <QGLWidget>
#include <QInputDialog>

using namespace csapex;

DesignerView::DesignerView(DesignerScene *scene, csapex::GraphPtr graph,
                           Settings &settings, ThreadPool &thread_pool,
                           CommandDispatcher *dispatcher, WidgetControllerPtr widget_ctrl, DragIO& dragio,
                           QWidget *parent)
    : QGraphicsView(parent), scene_(scene), settings_(settings),
      thread_pool_(thread_pool), graph_(graph), dispatcher_(dispatcher), widget_ctrl_(widget_ctrl), drag_io_(dragio),
      scalings_to_perform_(0)

{
    setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers))); // memory leak?
    setViewportUpdateMode(QGraphicsView::FullViewportUpdate);

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

    setContextMenuPolicy(Qt::DefaultContextMenu);
}

DesignerView::~DesignerView()
{
    delete scene_;
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

void DesignerView::updateSelection()
{
    QList<QGraphicsItem *> selected = scene_->items();
    foreach(QGraphicsItem* item, selected) {
        MovableGraphicsProxyWidget* m = dynamic_cast<MovableGraphicsProxyWidget*>(item);
        if(m) {
            m->getBox()->setSelected(m->isSelected());
        }
    }
}

Command::Ptr DesignerView::deleteSelected()
{
    command::Meta::Ptr meta(new command::Meta("delete selected boxes"));
    foreach(QGraphicsItem* item, scene_->selectedItems()) {
        MovableGraphicsProxyWidget* proxy = dynamic_cast<MovableGraphicsProxyWidget*>(item);
        if(proxy) {
            meta->add(Command::Ptr(new command::DeleteNode(proxy->getBox()->getNode()->getUUID())));
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
    QGraphicsView::dragEnterEvent(e);
    drag_io_.dragEnterEvent(this, e);
}

void DesignerView::dragMoveEvent(QDragMoveEvent* e)
{
    QGraphicsView::dragMoveEvent(e);
    if(!e->isAccepted()) {
        drag_io_.dragMoveEvent(this, e);
    }
}

void DesignerView::dropEvent(QDropEvent* e)
{
    QGraphicsView::dropEvent(e);
    drag_io_.dropEvent(this, e, mapToScene(e->pos()));
}

void DesignerView::dragLeaveEvent(QDragLeaveEvent* e)
{
    QGraphicsView::dragLeaveEvent(e);
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
            dispatcher_->executeLater(Command::Ptr(new command::AddNode(type, pos.toPoint(), UUID::NONE, uuid, NodeStateNullPtr)));
        }
    }
}

void DesignerView::addBoxEvent(NodeBox *box)
{
    QObject::connect(box, SIGNAL(renameRequest(NodeBox*)), this, SLOT(renameBox(NodeBox*)));

    QObject::connect(box, SIGNAL(moved(NodeBox*, int, int)), scene_, SLOT(boxMoved(NodeBox*)));

    Node* node = box->getNode();
    NodeWorker* worker = node->getNodeWorker();
    QObject::connect(worker, SIGNAL(connectionStart(Connectable*)), scene_, SLOT(deleteTemporaryConnections()), Qt::QueuedConnection);
    QObject::connect(worker, SIGNAL(connectionInProgress(Connectable*,Connectable*)), scene_, SLOT(previewConnection(Connectable*,Connectable*)), Qt::QueuedConnection);
    QObject::connect(worker, SIGNAL(connectionDone(Connectable*)), scene_, SLOT(deleteTemporaryConnectionsAndRepaint()), Qt::QueuedConnection);

    QObject::connect(graph_.get(), SIGNAL(structureChanged(Graph*)), box, SLOT(updateBoxInformation(Graph*)));

    QObject::connect(box, SIGNAL(showContextMenuForBox(NodeBox*, QPoint)), this, SLOT(showContextMenuEditBox(NodeBox*, QPoint)));
    QObject::connect(box, SIGNAL(profile(NodeBox*)), this, SLOT(profile(NodeBox*)));
    QObject::connect(box, SIGNAL(stopProfiling(NodeBox*)), this, SLOT(stopProfiling(NodeBox*)));

    MovableGraphicsProxyWidget* proxy = widget_ctrl_->getProxy(box->getNode()->getUUID());
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
}

void DesignerView::profile(NodeBox *box)
{
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

    MovableGraphicsProxyWidget* proxy = widget_ctrl_->getProxy(box->getNode()->getUUID());
    QObject::connect(proxy, SIGNAL(moving(double,double)), prof, SLOT(reposition(double,double)));
    QObject::connect(box->getNode()->getNodeWorker(), SIGNAL(messageProcessed()), prof, SLOT(repaint()));
}

void DesignerView::stopProfiling(NodeBox *box)
{
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
            meta->add(Command::Ptr(new command::MoveBox(b->getNode()->getUUID(), from, to)));
        }
    }

    dispatcher_->execute(meta);
}

void DesignerView::overwriteStyleSheet(QString &stylesheet)
{
    setStyleSheet(stylesheet);

    scene_->setInputColor(input_color_);
    scene_->setOutputColor(output_color_);

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

void DesignerView::showContextMenuEditBox(NodeBox* box, const QPoint &scene_pos)
{
    QMenu menu;
    std::map<QAction*, boost::function<void()> > handler;

    ContextMenuHandler::addHeader(menu, std::string("Node: ") + box->getNodeWorker()->getNodeUUID().getShortName());

    if(box->isMinimizedSize()) {
        QAction* max = new QAction("maximize", &menu);
        max->setIcon(QIcon(":/maximize.png"));
        max->setIconVisibleInMenu(true);
        handler[max] = boost::bind(&NodeBox::minimizeBox, box, false);
        menu.addAction(max);

    } else {
        QAction* min = new QAction("minimize", &menu);
        min->setIcon(QIcon(":/minimize.png"));
        min->setIconVisibleInMenu(true);
        handler[min] = boost::bind(&NodeBox::minimizeBox, box, true);
        menu.addAction(min);
    }

    QAction* flip = new QAction("flip sides", &menu);
    flip->setIcon(QIcon(":/flip.png"));
    flip->setIconVisibleInMenu(true);
    handler[flip] = boost::bind(&NodeBox::flipSides, box);
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
        handler[private_thread] = boost::bind(&ThreadPool::usePrivateThreadFor, &thread_pool_,  box->getNodeWorker());
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
            handler[switch_thread] = boost::bind(&ThreadPool::switchToThread, &thread_pool_, box->getNodeWorker(), group.id);
            choose_group_menu->addAction(switch_thread);
        }

        choose_group_menu->addSeparator();

        QAction* new_group = new QAction("new thread group", &menu);
        new_group->setIcon(QIcon(":/thread_group_add.png"));
        new_group->setIconVisibleInMenu(true);
        //        handler[new_group] = boost::bind(&ThreadPool::createNewThreadGroupFor, &thread_pool_,  box->getNodeWorker());
        handler[new_group] = boost::bind(&DesignerView::createNewThreadGroupFor, this,  box->getNodeWorker());

        choose_group_menu->addAction(new_group);

        thread_menu.addMenu(choose_group_menu);
    }

    QAction* term = new QAction("terminate thread", &menu);
    term->setIcon(QIcon(":/stop.png"));
    term->setIconVisibleInMenu(true);
    handler[term] = boost::bind(&NodeBox::killContent, box);
    menu.addAction(term);

    menu.addSeparator();

    QAction* prof;
    if(box->isProfiling()) {
        prof = new QAction("stop profiling", &menu);
        prof->setIcon(QIcon(":/stop_profiling.png"));
    } else {
        prof = new QAction("profiling", &menu);
        prof->setIcon(QIcon(":/profiling.png"));
    }
    prof->setIconVisibleInMenu(true);
    handler[prof] = boost::bind(&NodeBox::showProfiling, box);
    menu.addAction(prof);

    QAction* info = new QAction("get information", &menu);
    info->setIcon(QIcon(":/help.png"));
    info->setIconVisibleInMenu(true);
    handler[info] = boost::bind(&NodeBox::getInformation, box);
    menu.addAction(info);

    menu.addSeparator();

    QAction* del = new QAction("delete", &menu);
    del->setIcon(QIcon(":/close.png"));
    del->setIconVisibleInMenu(true);
    handler[del] = boost::bind(&DesignerView::deleteBox, this, box);
    menu.addAction(del);

    QAction* selectedItem = menu.exec(mapToGlobal(mapFromScene(scene_pos)));

    if(selectedItem) {
        handler[selectedItem]();
    }
}

void DesignerView::createNewThreadGroupFor(NodeWorker* worker)
{
    bool ok;
    QString text = QInputDialog::getText(this, "Group Name", "Enter new name", QLineEdit::Normal, QString::fromStdString(thread_pool_.nextName()), &ok);

    if(ok && !text.isEmpty()) {
        thread_pool_.createNewThreadGroupFor(worker, text.toStdString());
    }
}

void DesignerView::deleteBox(NodeBox* box)
{
    dispatcher_->execute(Command::Ptr(new command::DeleteNode(box->getNodeWorker()->getNodeUUID())));
}

void DesignerView::contextMenuEvent(QContextMenuEvent* event)
{
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
        widget_ctrl_->startPlacingBox(this, selected, NodeStateNullPtr);
    }
}

void DesignerView::selectAll()
{
    foreach(QGraphicsItem* item, scene_->items()) {
        item->setSelected(true);
    }
}
