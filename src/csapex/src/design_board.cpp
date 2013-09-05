/// HEADER
#include <csapex/design_board.h>

/// PROJECT
#include "ui_design_board.h"
#include <csapex/connector.h>
#include <csapex/selector_proxy.h>
#include <csapex/box.h>
#include <csapex/box_group.h>
#include <csapex/connector_in.h>
#include <csapex/connector_out.h>
#include <csapex/command_add_box.h>
#include <csapex/drag_io.h>
#include <csapex/box_manager.h>
#include <csapex/command_dispatcher.h>
#include <csapex/overlay.h>
#include <csapex/graph.h>

/// SYSTEM
#include <QResizeEvent>
#include <QMenu>
#include <iostream>
#include <QDropEvent>
#include <QDragEnterEvent>
#include <QScrollArea>
#include <QScrollBar>


using namespace csapex;

DesignBoard::DesignBoard(QWidget* parent)
    : QWidget(parent), ui(new Ui::DesignBoard), drag_io(DragIO::instance()), space_(false), drag_(false)
{
    ui->setupUi(this);

    overlay = new Overlay(this);

    installEventFilter(this);

    setMouseTracking(true);
    setContextMenuPolicy(Qt::CustomContextMenu);

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
    QObject::connect(box, SIGNAL(moved(Box*, int, int)), Graph::root().get(), SLOT(boxMoved(Box*, int, int)));
    QObject::connect(box, SIGNAL(changed(Box*)), overlay, SLOT(invalidateSchema()));
    QObject::connect(box, SIGNAL(clicked(Box*)), Graph::root().get(), SLOT(toggleBoxSelection(Box*)));
    QObject::connect(box, SIGNAL(connectionStart()), overlay, SLOT(deleteTemporaryConnections()));
    QObject::connect(box, SIGNAL(connectionInProgress(Connector*,Connector*)), overlay, SLOT(addTemporaryConnection(Connector*,Connector*)));
    QObject::connect(box, SIGNAL(connectionDone()), overlay, SLOT(deleteTemporaryConnectionsAndRepaint()));

    box->setParent(this);
    box->show();
    box->triggerPlaced();

    overlay->raise();
    repaint();
}

void DesignBoard::refresh()
{
    overlay->refresh();
    overlay->raise();
}

void DesignBoard::keyPressEvent(QKeyEvent* e)
{
    if(!overlay->keyPressEventHandler(e)) {
        return;
    }

    if(e->key() == Qt::Key_Space) {
        space_ = true;
    }
}

void DesignBoard::keyReleaseEvent(QKeyEvent* e)
{
    Graph::Ptr graph_ = Graph::root();
    // BOXES
    if(e->key() == Qt::Key_Delete || e->key() == Qt::Key_Backspace) {
        if(graph_->noSelectedBoxes() != 0) {
            graph_->deleteSelectedBoxes();
            return;
        }
    } else  if(e->key() == Qt::Key_G && Qt::ControlModifier == QApplication::keyboardModifiers()) {
        if(graph_->noSelectedBoxes() != 0) {
            graph_->groupSelectedBoxes();
            return;
        }
    }

    if(!overlay->keyReleaseEventHandler(e)) {
        return;
    }

    if(!e->isAutoRepeat() && e->key() == Qt::Key_Space) {
        space_ = false;
        drag_ = false;
    }
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

    Graph::Ptr graph_ = Graph::root();

    if(e->button() == Qt::LeftButton) {
        QRect selection(mapFromGlobal(drag_start_pos_), mapFromGlobal(e->globalPos()));
        if(std::abs(selection.width()) > 5 && std::abs(selection.height()) > 5) {
            graph_->deselectBoxes();

            foreach(csapex::Box* box, findChildren<csapex::Box*>()) {
                if(selection.contains(box->geometry())) {
                    graph_->selectBox(box, true);
                }
            }

            return;
        }
    }

    // BOXES
    bool shift = Qt::ShiftModifier == QApplication::keyboardModifiers();
    if(!shift) {
        graph_->deselectBoxes();
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

            QScrollArea* parent_scroll = NULL;
            QWidget* tmp = parentWidget();
            while(tmp != NULL) {
                parent_scroll = dynamic_cast<QScrollArea*>(tmp);
                if(parent_scroll) {
                    break;
                }
                tmp = tmp->parentWidget();
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
    }

    if(!overlay->mouseMoveEventHandler(drag_, e)) {
        return;
    }
    if(drag_ && !space_) {
        overlay->setSelectionRectangle(overlay->mapFromGlobal(drag_start_pos_), overlay->mapFromGlobal(e->globalPos()));
        overlay->repaint();
    }
}

bool DesignBoard::eventFilter(QObject*, QEvent*)
{
    return false;
}


void DesignBoard::showContextMenu(const QPoint& pos)
{
    if(overlay->showContextMenu(pos)) {
        return;
    }

    QPoint globalPos = mapToGlobal(pos);

    QMenu menu;
    BoxManager::instance().insertAvailableBoxedObjects(&menu);

    QAction* selectedItem = menu.exec(globalPos);

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
