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

/// SYSTEM
#include <iostream>
#include <QKeyEvent>
#include <QApplication>
#include <QShortcut>
#include <QGraphicsItem>
#include <QGLWidget>

using namespace csapex;

DesignerView::DesignerView(csapex::GraphPtr graph, CommandDispatcher *dispatcher, WidgetControllerPtr widget_ctrl, DragIO& dragio, QWidget *parent)
    : QGraphicsView(parent), graph_(graph), dispatcher_(dispatcher), widget_ctrl_(widget_ctrl), drag_io_(dragio)

{
    scene_ = new DesignerScene(graph_, dispatcher_, widget_ctrl_);
    setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
    setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    setScene(scene_);
    setFocusPolicy(Qt::StrongFocus);
    setFocus(Qt::OtherFocusReason);

    setAcceptDrops(true);

    setDragMode(QGraphicsView::RubberBandDrag);
    setInteractive(true);

    QShortcut *shortcut = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_Space), this);
    QObject::connect(shortcut, SIGNAL(activated()), this, SLOT(showBoxDialog()));

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

DesignerScene* DesignerView::designerScene()
{
    return scene_;
}

bool DesignerView::hasSelection() const
{
    return scene_->selectedItems().size() > 0;
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
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

    double scaleFactor = 1.15;
    if(we->delta() > 0) {
        // Zoom in
        scale(scaleFactor, scaleFactor);
    } else {
        // Zooming out
        scale(1.0 / scaleFactor, 1.0 / scaleFactor);
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
    //QObject::connect(box, SIGNAL(moved(csapex::Box*, int, int)), this, SLOT(findMinSize(csapex::Box*)));
    //QObject::connect(box, SIGNAL(moved(Box*, int, int)), overlay_, SLOT(invalidateSchema()));
    //QObject::connect(box, SIGNAL(moved(Box*, int, int)), &widget_ctrl_->box_selection_, SLOT(mimicBoxMovement(Box*, int, int)));
    //QObject::connect(box, SIGNAL(changed(Box*)), overlay_, SLOT(invalidateSchema()));
    //QObject::connect(box, SIGNAL(clicked(Box*)), &widget_ctrl_->box_selection_, SLOT(toggleSelection(Box*)));
    QObject::connect(box->getNode(), SIGNAL(connectionStart()), scene_, SLOT(deleteTemporaryConnections()));
    QObject::connect(box->getNode(), SIGNAL(connectionInProgress(Connectable*,Connectable*)), scene_, SLOT(addTemporaryConnection(Connectable*,Connectable*)));
    QObject::connect(box->getNode(), SIGNAL(connectionDone()), scene_, SLOT(deleteTemporaryConnectionsAndRepaint()));

    QObject::connect(graph_.get(), SIGNAL(structureChanged(Graph*)), box, SLOT(updateInformation(Graph*)));

    QObject::connect(box, SIGNAL(showContextMenuForBox(NodeBox*, QPoint)), this, SLOT(showContextMenuEditBox(NodeBox*, QPoint)));

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

    //overlay_->raise();
    //repaint();
}

void DesignerView::removeBoxEvent(NodeBox *box)
{
    box->setVisible(false);
    box->deleteLater();
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

void DesignerView::enableGrid(bool draw)
{
    scene_->enableGrid(draw);
}
