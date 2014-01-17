/// HEADER
#include <csapex/view/design_board.h>

/// PROJECT
#include "ui_design_board.h"
#include <csapex/model/connectable.h>
#include <csapex/model/node_constructor.h>
#include <csapex/view/box.h>
#include <csapex/model/group.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/command/add_node.h>
#include <csapex/core/drag_io.h>
#include <csapex/manager/box_manager.h>
#include <csapex/command/dispatcher.h>
#include <csapex/view/overlay.h>
#include <csapex/model/graph.h>
#include <csapex/view/box_dialog.h>

/// SYSTEM
#include <QResizeEvent>
#include <QMenu>
#include <iostream>
#include <QDropEvent>
#include <QDragEnterEvent>
#include <QScrollArea>
#include <QScrollBar>
#include <QShortcut>

using namespace csapex;

DesignBoard::DesignBoard(CommandDispatcher* dispatcher, QWidget* parent)
    : QWidget(parent), ui(new Ui::DesignBoard), dispatcher_(dispatcher), drag_io(dispatcher),
      space_(false), drag_(false), parent_scroll(NULL), initial_pos_x_(0), initial_pos_y_(0)
{
    ui->setupUi(this);

    QShortcut *shortcut = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_Space), this);
    QObject::connect(shortcut, SIGNAL(activated()), this, SLOT(showBoxDialog()));

    overlay = new Overlay(dispatcher, this);

    installEventFilter(this);

    setMouseTracking(true);
    setContextMenuPolicy(Qt::CustomContextMenu);

    setFocusPolicy(Qt::StrongFocus);
    setFocus(Qt::OtherFocusReason);

    BoxManager::instance().setContainer(this);

    connect(this, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(showContextMenu(const QPoint&)));
}

void DesignBoard::enableGrid(bool grid)
{
    setProperty("grid", grid);

    style()->unpolish(this);
    style()->polish(this);
}

DesignBoard::~DesignBoard()
{}

void DesignBoard::updateCursor()
{
    if(space_) {
        if(drag_) {
            setCursor(Qt::ClosedHandCursor);
        } else {
            setCursor(Qt::OpenHandCursor);
        }
    } else {
        setCursor(Qt::ArrowCursor);
    }
}

void DesignBoard::paintEvent(QPaintEvent*)
{
    if(initial_pos_x_ != 0 || initial_pos_y_ != 0) {
        if(!parent_scroll) {
            findParentScroll();
        }

        if(parent_scroll) {
            QSize minimum = size();
            minimum.setWidth(minimum.width() + std::abs(initial_pos_x_));
            minimum.setHeight(minimum.height() + std::abs(initial_pos_y_));

            setMinimumSize(minimum);

            parent_scroll->horizontalScrollBar()->setValue(initial_pos_x_);
            parent_scroll->verticalScrollBar()->setValue(initial_pos_y_);

            int vx = parent_scroll->horizontalScrollBar()->value();
            int vy = parent_scroll->verticalScrollBar()->value();

            if(initial_pos_x_ == vx && initial_pos_y_ == vy) {
                initial_pos_x_ = 0;
                initial_pos_y_ = 0;
            }
        }
    }

    QStyleOption opt;
    opt.init(this);
    QPainter p(this);
    style()->drawPrimitive(QStyle::PE_Widget, &opt, &p, this);

    updateCursor();
}

void DesignBoard::findMinSize(Box* box)
{
    QSize minimum = minimumSize();

    minimum.setWidth(std::max(minimum.width(), box->pos().x() + box->width()));
    minimum.setHeight(std::max(minimum.height(), box->pos().y() + box->height()));

    QPoint pos = box->pos();

    if(pos.x() < 0 || pos.y() < 0) {
        if(pos.x() < 0) {
            pos.setX(0);
        }
        if(pos.y() < 0) {
            pos.setY(0);
        }
        box->move(pos);
    }

    setMinimumSize(minimum);
}

void DesignBoard::addBoxEvent(Box *box)
{
    QObject::connect(box, SIGNAL(moved(Box*, int, int)), this, SLOT(findMinSize(Box*)));
    QObject::connect(box, SIGNAL(moved(Box*, int, int)), overlay, SLOT(invalidateSchema()));
    QObject::connect(box, SIGNAL(moved(Box*, int, int)), dispatcher_->getGraph().get(), SLOT(boxMoved(Box*, int, int)));
    QObject::connect(box, SIGNAL(changed(Box*)), overlay, SLOT(invalidateSchema()));
    QObject::connect(box, SIGNAL(clicked(Box*)), dispatcher_->getGraph().get(), SLOT(toggleBoxSelection(Box*)));
    QObject::connect(box->getNode(), SIGNAL(connectionStart()), overlay, SLOT(deleteTemporaryConnections()));
    QObject::connect(box->getNode(), SIGNAL(connectionInProgress(Connectable*,Connectable*)), overlay, SLOT(addTemporaryConnection(Connectable*,Connectable*)));
    QObject::connect(box->getNode(), SIGNAL(connectionDone()), overlay, SLOT(deleteTemporaryConnectionsAndRepaint()));

    QObject::connect(box, SIGNAL(showContextMenuForBox(Box*, QPoint)), this, SLOT(showContextMenuEditBox(Box*, QPoint)));

    box->setParent(this);
    box->init();
    box->triggerPlaced();
    box->setCommandDispatcher(dispatcher_);
    box->show();

    overlay->raise();
    repaint();
}

void DesignBoard::removeBoxEvent(Box *box)
{
    box->deleteLater();
}

void DesignBoard::refresh()
{
    overlay->refresh();
    overlay->raise();
}

void DesignBoard::reset()
{
    bool has_child = true;
    while(has_child) {
        Box* b = findChild<Box*>();
        has_child = b != NULL;
        if(has_child) {
            delete b;
        }
    }
}

void DesignBoard::setSpace(bool s)
{
    space_ = s;

    overlay->blockMouse(space_);

    //    Q_FOREACH(csapex::Box* box, findChildren<csapex::Box*>()) {
    //        box->setEnabled(!space_);
    //    }
}

void DesignBoard::keyPressEvent(QKeyEvent* e)
{
    if(!overlay->keyPressEventHandler(e)) {
        return;
    }

    if(e->key() == Qt::Key_Space && Qt::ControlModifier != QApplication::keyboardModifiers()) {
        setSpace(true);
    }
}

void DesignBoard::showBoxDialog()
{
    BoxDialog diag;
    int r = diag.exec();

    if(r) {
        //BoxManager::instance().startPlacingBox(this, diag.getName());

        std::string type = diag.getName();
        UUID uuid = UUID::make(dispatcher_->getGraph()->makeUUIDPrefix(type));
        QPoint pos = mapFromGlobal(QCursor::pos());
        dispatcher_->executeLater(Command::Ptr(new command::AddNode(type, pos, UUID::NONE, uuid, NodeStateNullPtr)));
    }
}

void DesignBoard::keyReleaseEvent(QKeyEvent* e)
{
    Graph::Ptr graph = dispatcher_->getGraph();

    // BOXES
    if(Qt::ControlModifier == QApplication::keyboardModifiers()) {
        if(e->key() == Qt::Key_Delete || e->key() == Qt::Key_Backspace) {
            if(graph->countSelectedNodes() != 0) {
                dispatcher_->execute(graph->deleteSelectedNodesCmd());
                return;
            }
        } else  if(e->key() == Qt::Key_G) {
            if(graph->countSelectedNodes() != 0) {
                dispatcher_->execute(graph->groupSelectedNodesCmd());
                return;
            }
        }
    }

    if(!overlay->keyReleaseEventHandler(e)) {
        return;
    }

    if(!e->isAutoRepeat() && e->key() == Qt::Key_Space) {
        setSpace(false);
        drag_ = false;
    }
}

void DesignBoard::wheelEvent(QWheelEvent *e)
{
    e->accept();
    setFocus();
}

void DesignBoard::mousePressEvent(QMouseEvent* e)
{
    if(!drag_ || !space_) {
        if(!overlay->mousePressEventHandler(e)) {
            return;
        }
    }

    if(e->button() == Qt::LeftButton) {
        drag_ = true;
        drag_start_pos_ = e->globalPos();
        updateCursor();
    }
}

void DesignBoard::mouseReleaseEvent(QMouseEvent* e)
{

    if(e->button() == Qt::LeftButton) {
        drag_ = false;
        overlay->setSelectionRectangle(QPoint(),QPoint());
    }

    if(!overlay->mouseReleaseEventHandler(e)) {
        return;
    }

    if(e->button() == Qt::LeftButton) {
        QRect selection(mapFromGlobal(drag_start_pos_), mapFromGlobal(e->globalPos()));
        if(std::abs(selection.width()) > 5 && std::abs(selection.height()) > 5) {
            dispatcher_->getGraph()->deselectNodes();

            Q_FOREACH(csapex::Box* box, findChildren<csapex::Box*>()) {
                if(selection.contains(box->geometry())) {
                    dispatcher_->getGraph()->selectNode(box->getNode(), true);
                }
            }

            return;
        }
    }

    // BOXES
    bool shift = Qt::ShiftModifier == QApplication::keyboardModifiers();
    if(!shift) {
        dispatcher_->getGraph()->deselectNodes();
    }
    updateCursor();
}

void DesignBoard::mouseMoveEvent(QMouseEvent* e)
{
    if(drag_) {
        if( space_) {
            QSize minimum = minimumSize();
            if(minimum.width() < size().width()) {
                minimum.setWidth(size().width());
            }
            if(minimum.height() < size().height()) {
                minimum.setHeight(size().height());
            }

            setMinimumSize(minimum);

            updateCursor();
            QPoint delta = e->globalPos() - drag_start_pos_;
            drag_start_pos_ = e->globalPos();

            scrollBy(delta.x(), delta.y());
        }
    }

    if(!overlay->mouseMoveEventHandler(drag_, e)) {
        return;
    }
    if(drag_ && !space_) {
        overlay->setSelectionRectangle(overlay->mapFromGlobal(drag_start_pos_), overlay->mapFromGlobal(e->globalPos()));
        overlay->repaint();
    }

    if(!drag_) {
        setFocus();
    }
}

void DesignBoard::setView(int sx, int sy)
{
    initial_pos_x_ = sx;
    initial_pos_y_ = sy;
}

void DesignBoard::findParentScroll()
{
    QWidget* tmp = parentWidget();
    while(tmp != NULL) {
        parent_scroll = dynamic_cast<QScrollArea*>(tmp);
        if(parent_scroll) {
            break;
        }
        tmp = tmp->parentWidget();
    }
}

void DesignBoard::scrollBy(int x, int y)
{
    QPoint delta(x, y);

    if(!parent_scroll) {
        findParentScroll();
    }

    if(parent_scroll) {
        int sbh = parent_scroll->horizontalScrollBar()->value();
        int sbv = parent_scroll->verticalScrollBar()->value();

        parent_scroll->horizontalScrollBar()->setValue(sbh - delta.x());
        parent_scroll->verticalScrollBar()->setValue(sbv - delta.y());

        int sbh_after = parent_scroll->horizontalScrollBar()->value();
        int sbv_after = parent_scroll->verticalScrollBar()->value();

        if(delta.x() > 0) {
            delta.setX(0);
        }
        if(delta.y() > 0) {
            delta.setY(0);
        }
        int dx = sbh - delta.x() - sbh_after;
        int dy = sbv - delta.y() - sbv_after;

        if(dx != 0 || dy != 0) {
            QSize minimum = minimumSize();

            minimum.setWidth(minimum.width() + std::abs(dx));
            minimum.setHeight(minimum.height() + std::abs(dy));

            setMinimumSize(minimum);
        }
    }
}

void DesignBoard::focusInEvent(QFocusEvent *)
{
    setProperty("focused", true);
    setStyleSheet(styleSheet());
}


void DesignBoard::focusOutEvent(QFocusEvent *)
{
    setProperty("focused", false);
    setStyleSheet(styleSheet());
}

bool DesignBoard::eventFilter(QObject*, QEvent*)
{
    return false;
}


void DesignBoard::showContextMenuGlobal(const QPoint& global_pos)
{
    if(overlay->showContextMenu(global_pos)) {
        return;
    }

    /// BOXES
    dispatcher_->getGraph()->deselectNodes();
    showContextMenuAddNode(global_pos);
}

void DesignBoard::showContextMenuEditBox(Box* box, const QPoint &global_pos)
{
    QMenu menu;
    std::map<QAction*, boost::function<void()> > handler;

    Graph::Ptr graph = dispatcher_->getGraph();

    if(box != NULL && !box->isSelected()) {
        graph->deselectNodes();
        box->setSelected(true);
    }

    if(graph->countSelectedNodes() == 1) {
        box->fillContextMenu(&menu, handler);

    } else {
        graph->fillContextMenuForSelection(&menu, handler);
    }

    QAction* selectedItem = menu.exec(global_pos);

    if(selectedItem) {
        handler[selectedItem]();
    }
}

void DesignBoard::showContextMenu(const QPoint& pos)
{
    showContextMenuGlobal(mapToGlobal(pos));
}

void DesignBoard::showContextMenuAddNode(const QPoint &global_pos)
{
    QMenu menu;
    BoxManager::instance().insertAvailableBoxedObjects(&menu);

    QAction* selectedItem = menu.exec(global_pos);

    if(selectedItem) {
        std::string selected = selectedItem->data().toString().toStdString();
        BoxManager::instance().startPlacingBox(this, selected);
    }
}

void DesignBoard::resizeEvent(QResizeEvent* e)
{
    overlay->resize(e->size());
}

void DesignBoard::dragEnterEvent(QDragEnterEvent* e)
{
    drag_io.dragEnterEvent(this, overlay, e);
}

void DesignBoard::dragMoveEvent(QDragMoveEvent* e)
{
    drag_io.dragMoveEvent(this, overlay, e);
}

void DesignBoard::dropEvent(QDropEvent* e)
{
    drag_io.dropEvent(this, overlay, e);
}

void DesignBoard::dragLeaveEvent(QDragLeaveEvent*)
{
}

void DesignBoard::enterEvent(QEvent *)
{
    setFocus();
}
