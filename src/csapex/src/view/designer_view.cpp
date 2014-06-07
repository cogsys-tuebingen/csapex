/// HEADER
#include <csapex/view/designer_view.h>

/// COMPONENT
#include <csapex/command/meta.h>
#include <csapex/command/move_box.h>
#include <csapex/command/delete_node.h>
#include <csapex/view/box_dialog.h>
#include <csapex/manager/box_manager.h>
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

/// SYSTEM
#include <iostream>
#include <QKeyEvent>
#include <QApplication>
#include <QShortcut>
#include <QGraphicsItem>
#include <QGLWidget>

using namespace csapex;

DesignerView::DesignerView(DesignerScene *scene, csapex::GraphPtr graph, CommandDispatcher *dispatcher, WidgetControllerPtr widget_ctrl, DragIO& dragio, QWidget *parent)
    : QGraphicsView(parent), scene_(scene), graph_(graph), dispatcher_(dispatcher), widget_ctrl_(widget_ctrl), drag_io_(dragio)

{
    setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
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

    QObject::connect(scene_, SIGNAL(selectionChanged()), this, SLOT(updateSelection()));
    QObject::connect(scene_, SIGNAL(selectionChanged()), this, SIGNAL(selectionChanged()));

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

DesignerScene* DesignerView::designerScene()
{
    return scene_;
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
        we->accept();

        setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

        double scaleFactor = shift ? 1.05 : 1.25;
        if(we->delta() > 0) {
            // Zoom in
            scale(scaleFactor, scaleFactor);
        } else {
            // Zooming out
            scale(1.0 / scaleFactor, 1.0 / scaleFactor);
        }

        scene_->setScale(transform().m11());

    } else {
        QGraphicsView::wheelEvent(we);
    }
}


void DesignerView::dragEnterEvent(QDragEnterEvent* e)
{
    QGraphicsView::dragEnterEvent(e);
    drag_io_.dragEnterEvent(this, e);
}

void DesignerView::dragMoveEvent(QDragMoveEvent* e)
{
    QGraphicsView::dragMoveEvent(e);
    drag_io_.dragMoveEvent(this, e);
}

void DesignerView::dropEvent(QDropEvent* e)
{
    QGraphicsView::dropEvent(e);
    drag_io_.dropEvent(this, e);
}

void DesignerView::dragLeaveEvent(QDragLeaveEvent* e)
{
    QGraphicsView::dragLeaveEvent(e);
}

void DesignerView::showBoxDialog()
{
    BoxDialog diag;
    int r = diag.exec();

    if(r) {
        //BoxManager::instance().startPlacingBox(this, diag.getName());

        std::string type = diag.getName();

        if(!type.empty() && BoxManager::instance().isValidType(type)) {
            UUID uuid = UUID::make(graph_->makeUUIDPrefix(type));
            QPointF pos = mapToScene(mapFromGlobal(QCursor::pos()));
            dispatcher_->executeLater(Command::Ptr(new command::AddNode(type, pos.toPoint(), UUID::NONE, uuid, NodeStateNullPtr)));
        }
    }
}

void DesignerView::addBoxEvent(NodeBox *box)
{
    QObject::connect(box, SIGNAL(moved(NodeBox*, int, int)), scene_, SLOT(invalidateSchema()));
    QObject::connect(box, SIGNAL(changed(NodeBox*)), scene_, SLOT(invalidateSchema()));

    QObject::connect(box->getNode(), SIGNAL(connectionStart()), scene_, SLOT(deleteTemporaryConnections()));
    QObject::connect(box->getNode(), SIGNAL(connectionInProgress(Connectable*,Connectable*)), scene_, SLOT(addTemporaryConnection(Connectable*,Connectable*)));
    QObject::connect(box->getNode(), SIGNAL(connectionDone()), scene_, SLOT(deleteTemporaryConnectionsAndRepaint()));

    QObject::connect(graph_.get(), SIGNAL(structureChanged(Graph*)), box, SLOT(updateInformation(Graph*)));

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
    box->setCommandDispatcher(dispatcher_);
    box->show();

    box->updateInformation(graph_.get());
}

void DesignerView::removeBoxEvent(NodeBox *box)
{
    box->setVisible(false);
    box->deleteLater();
}

void DesignerView::profile(NodeBox *box)
{
    assert(profiling_.find(box) == profiling_.end());

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
    assert(pos != profiling_.end());

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

void DesignerView::showContextMenuGlobal(const QPoint& global_pos)
{
    /// BOXES
    showContextMenuAddNode(global_pos);
}

void DesignerView::showContextMenuEditBox(NodeBox* box, const QPoint &global_pos)
{
    QMenu menu;
    std::map<QAction*, boost::function<void()> > handler;

    box->fillContextMenu(&menu, handler);

    QAction* selectedItem = menu.exec(mapToGlobal(mapFromScene(global_pos)));

    if(selectedItem) {
        handler[selectedItem]();
    }
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
    BoxManager::instance().insertAvailableNodeTypes(&menu);

    QAction* selectedItem = menu.exec(global_pos);

    if(selectedItem) {
        std::string selected = selectedItem->data().toString().toStdString();
        BoxManager::instance().startPlacingBox(this, selected, widget_ctrl_.get());
    }
}

void DesignerView::selectAll()
{
    foreach(QGraphicsItem* item, scene_->items()) {
        item->setSelected(true);
    }
}
