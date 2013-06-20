/// HEADER
#include "design_board.h"

/// PROJECT
#include "ui_design_board.h"
#include "connector.h"
#include "selector_proxy.h"
#include "box.h"
#include "command_add_box.h"
#include "box_manager.h"

/// SYSTEM
#include <boost/foreach.hpp>
#include <QResizeEvent>
#include <QMenu>
#include <iostream>
#include <QDropEvent>
#include <QDragEnterEvent>
#include <QScrollArea>
#include <QScrollBar>

using namespace vision_evaluator;

DesignBoard::DesignBoard(QWidget* parent)
    : QWidget(parent), ui(new Ui::DesignBoard), space_(false), drag_(false)
{
    ui->setupUi(this);

    overlay = new Overlay(this);

    installEventFilter(this);

    setContextMenuPolicy(Qt::CustomContextMenu);

    connect(this, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(showContextMenu(const QPoint&)));
}

DesignBoard::~DesignBoard()
{}

void DesignBoard::updateCursor()
{
    if(space_){
        if(drag_) {
            setCursor(Qt::ClosedHandCursor);
        } else {
            setCursor(Qt::OpenHandCursor);
        }
    } else {
        setCursor(Qt::ArrowCursor);
    }
}

void DesignBoard::paintEvent(QPaintEvent *)
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

    int movex = box->x() < 0 ? -box->x() : 0;
    int movey = box->y() < 0 ? -box->y() : 0;

    if(movex != 0 || movey != 0) {
        BOOST_FOREACH(vision_evaluator::Box* b, findChildren<vision_evaluator::Box*>()) {
            if(b != box) {
                b->move(b->x() + movex, b->y() + movey);
            }
        }
        minimum.setWidth(minimum.width() + movex);
        minimum.setHeight(minimum.height() + movey);
    }

    setMinimumSize(minimum);
}

void DesignBoard::keyPressEvent(QKeyEvent *e)
{
    if(e->key() == Qt::Key_Space) {
        space_ = true;
    }
}

void DesignBoard::keyReleaseEvent(QKeyEvent *e)
{
    if(!e->isAutoRepeat() && e->key() == Qt::Key_Space) {
        space_ = false;
        drag_ = false;
    }
}

void DesignBoard::mousePressEvent(QMouseEvent *e)
{
    drag_ = true;
    drag_start_pos_ = e->globalPos();
    updateCursor();
}

void DesignBoard::mouseReleaseEvent(QMouseEvent *e)
{
    drag_ = false;
    updateCursor();
}

void DesignBoard::mouseMoveEvent(QMouseEvent *e)
{
    if(drag_ && space_) {
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
            parent_scroll = dynamic_cast<QScrollArea*> (tmp);
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

            int dx = sbh - delta.x() - sbh_after;
            int dy = sbv - delta.y() - sbv_after;

            if(dx != 0 || dy != 0) {
                QSize minimum = minimumSize();

                minimum.setWidth(minimum.width() + std::abs(dx));
                minimum.setHeight(minimum.height() + std::abs(dy));

                int movex = dx < 0 ? -dx : 0;
                int movey = dy < 0 ? -dy : 0;

                if(movex != 0 || movey != 0) {
                    BOOST_FOREACH(vision_evaluator::Box* box, findChildren<vision_evaluator::Box*>()) {
                        box->move(box->x() + movex, box->y() + movey);
                    }
                }

                setMinimumSize(minimum);
            }
        }
    }
}

bool DesignBoard::eventFilter(QObject* o, QEvent* e)
{
    if(e->type() == QEvent::ChildPolished) {
        QChildEvent* ch = dynamic_cast<QChildEvent*>(e);
        QObject* child = ch->child();
        Box* box = dynamic_cast<Box*>(child);
        if(box) {
            box->setOverlay(overlay);

            findMinSize(box);

            QObject::connect(box, SIGNAL(moved(Box*)), this, SLOT(findMinSize(Box*)));
        }

        overlay->raise();
    }

    return false;
}


void DesignBoard::showContextMenu(const QPoint& pos)
{

}

void DesignBoard::resizeEvent(QResizeEvent* e)
{
    overlay->resize(e->size());
}

void DesignBoard::dragEnterEvent(QDragEnterEvent* e)
{
    if(e->mimeData()->text() == Box::MIME) {
        e->acceptProposedAction();
    }
    if(e->mimeData()->text() == Box::MIME_MOVE) {
        e->acceptProposedAction();
    }
    if(e->mimeData()->text() == Connector::MIME) {
        e->acceptProposedAction();
    }
}
void DesignBoard::dragMoveEvent(QDragMoveEvent* e)
{
    if(e->mimeData()->text() == Connector::MIME) {
        Connector* c = dynamic_cast<Connector*>(e->mimeData()->parent());
        overlay->drawTemporaryLine(QLine(c->centerPoint(), e->pos()));
    }

    if(e->mimeData()->text() == Box::MIME_MOVE) {
        Box* box = dynamic_cast<Box*>(e->mimeData()->parent());
        Box::MoveOffset* offset = dynamic_cast<Box::MoveOffset*>(e->mimeData()->userData(0));
        box->move(e->pos() + offset->value);
        overlay->repaint();
    }
}

void DesignBoard::dropEvent(QDropEvent* e)
{
    if(e->mimeData()->text() == Box::MIME) {
        SelectorProxy* selector = dynamic_cast<SelectorProxy*>(e->mimeData()->parent());

        if(!selector) {
            return;
        }

        e->setDropAction(Qt::CopyAction);
        e->accept();

        Command::Ptr add_box(new command::AddBox(selector, this, e->pos()));
        BoxManager::instance().execute(add_box);
    }

    if(e->mimeData()->text() == Connector::MIME) {
        e->ignore();
    }
}
