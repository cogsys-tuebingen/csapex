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
#include <QResizeEvent>
#include <QMenu>
#include <iostream>
#include <QDropEvent>
#include <QDragEnterEvent>

using namespace vision_evaluator;

DesignBoard::DesignBoard(QWidget* parent)
    : QWidget(parent), ui(new Ui::DesignBoard)
{
    ui->setupUi(this);

    overlay = new Overlay(this);

    installEventFilter(this);

    setContextMenuPolicy(Qt::CustomContextMenu);

    connect(this, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(showContextMenu(const QPoint&)));
}

DesignBoard::~DesignBoard()
{}


bool DesignBoard::eventFilter(QObject* o, QEvent* e)
{
    if(e->type() == QEvent::ChildPolished) {
        QChildEvent* ch = dynamic_cast<QChildEvent*>(e);
        QObject* child = ch->child();
        Box* box = dynamic_cast<Box*>(child);
        if(box) {
            box->setOverlay(overlay);
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

void DesignBoard::childEvent(QChildEvent *e)
{
    //if(e->type() == QChildEvent::ChildAdded) {
    //    resize(sizeHint());
    //}
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
