/// HEADER
#include <csapex/view/design_board.h>

/// PROJECT
#include <csapex/command/add_node.h>
#include <csapex/command/dispatcher.h>
#include <csapex/core/drag_io.h>
#include <csapex/manager/box_manager.h>
#include <csapex/model/connectable.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_constructor.h>
#include <csapex/model/node.h>
#include <csapex/view/box.h>
#include <csapex/view/overlay.h>
#include <csapex/view/widget_controller.h>
#include "ui_design_board.h"

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

DesignBoard::DesignBoard(Graph::Ptr graph, CommandDispatcher* dispatcher, WidgetControllerPtr widget_ctrl, DragIO& dragio, Overlay* overlay, QWidget* parent)
    : QWidget(parent), ui(new Ui::DesignBoard), graph_(graph), dispatcher_(dispatcher), widget_ctrl_(widget_ctrl), overlay_(overlay), drag_io_(dragio),
      space_(false), drag_(false), parent_scroll(NULL), initial_pos_x_(0), initial_pos_y_(0)
{
    ui->setupUi(this);

    overlay_->setParent(this);

    installEventFilter(this);

    setMouseTracking(true);
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

void DesignBoard::findMinSize(NodeBox* box)
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

void DesignBoard::refresh()
{
}

void DesignBoard::reset()
{
    bool has_child = true;
    while(has_child) {
        NodeBox* b = findChild<NodeBox*>();
        has_child = b != NULL;
        if(has_child) {
            delete b;
        }
    }
}

void DesignBoard::setSpace(bool s)
{
    space_ = s;

    overlay_->blockMouse(space_);

    //    Q_FOREACH(csapex::Box* box, findChildren<csapex::Box*>()) {
    //        box->setEnabled(!space_);
    //    }
}

void DesignBoard::keyPressEvent(QKeyEvent* e)
{
    if(!overlay_->keyPressEventHandler(e)) {
        return;
    }

    if(e->key() == Qt::Key_Space && Qt::ControlModifier != QApplication::keyboardModifiers()) {
        setSpace(true);
    }
}


void DesignBoard::keyReleaseEvent(QKeyEvent* e)
{
    if(!overlay_->keyReleaseEventHandler(e)) {
        return;
    }

    if(!e->isAutoRepeat() && e->key() == Qt::Key_Space) {
        setSpace(false);
        drag_ = false;
    }
}

void DesignBoard::mousePressEvent(QMouseEvent* e)
{
    if(!drag_ || !space_) {
        if(!overlay_->mousePressEventHandler(e)) {
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

    if(!overlay_->mouseMoveEventHandler(drag_, e)) {
        return;
    }
    if(drag_ && !space_) {
        overlay_->setSelectionRectangle(overlay_->mapFromGlobal(drag_start_pos_), overlay_->mapFromGlobal(e->globalPos()));
        overlay_->repaint();
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



void DesignBoard::resizeEvent(QResizeEvent* e)
{
    overlay_->resize(e->size());
}

void DesignBoard::enterEvent(QEvent *)
{
}
