/// HEADER
#include "design_board.h"

/// PROJECT
#include "ui_design_board.h"
#include "connector.h"
#include "selector_proxy.h"

/// SYSTEM
#include <QResizeEvent>
#include <QMenu>
#include <iostream>
#include <QDropEvent>
#include <QDragEnterEvent>

using namespace vision_evaluator;

DesignBoard::DesignBoard(QWidget *parent)
    : QWidget(parent), ui(new Ui::DesignBoard)
{
    ui->setupUi(this);

    overlay = new Overlay(this);

    installEventFilter(this);

    Box* test_box1 = new Box(this);
    test_box1->setGeometry(50, 100, 100, 100);

    Box* test_box2 = new Box(this);
    test_box2->setGeometry(240, 100, 100, 100);

    overlay->raise();

    setContextMenuPolicy(Qt::CustomContextMenu);

    connect(this, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(showContextMenu(const QPoint&)));
}

bool DesignBoard::eventFilter(QObject* o, QEvent *e)
{
    if(e->type() == QEvent::ChildPolished) {
        QChildEvent* ch = dynamic_cast<QChildEvent*>(e);
        QObject* child = ch->child();
        Box* box = dynamic_cast<Box*>(child);
        if(box) {
            box->setOverlay(overlay);
        }
    }

    return false;
}


void DesignBoard::showContextMenu(const QPoint &pos)
{
    QPoint globalPos = mapToGlobal(pos);

    QMenu myMenu;
    myMenu.addAction("Menu Item 1");

    QAction* selectedItem = myMenu.exec(globalPos);
    if (selectedItem)
    {
        // something was chosen, do stuff
    }
    else
    {
        // nothing was chosen
    }

}

void DesignBoard::connectorReleased(Connector* startConnector, const QPoint &pt)
{
    QWidget* target = childAt(mapFromGlobal(pt));
    Connector* connector = dynamic_cast<Connector*>(target);

    if(connector && connector != startConnector) {
        if(startConnector->tryConnect(connector)) {
            repaint();
        }
    }
}

void DesignBoard::resizeEvent(QResizeEvent *e)
{
    overlay->resize(e->size());
}

void DesignBoard::mouseReleaseEvent(QMouseEvent *e)
{
    QList<Box*> list = findChildren<Box*>();
    foreach(Box *box, list) {
        box->mouseReleaseEvent(e);
    }
}

void DesignBoard::dragEnterEvent(QDragEnterEvent *e)
{
    if (e->mimeData()->text() == QString("vision_evaluator/box")) {
        e->acceptProposedAction();
    }
}

void DesignBoard::dropEvent(QDropEvent *e)
{
    if (e->mimeData()->text() == QString("vision_evaluator/box")) {
        SelectorProxy* selector = dynamic_cast<SelectorProxy*> (e->mimeData()->parent());

        if(!selector) {
            return;
        }

        e->setDropAction(Qt::CopyAction);
        e->accept();

        selector->spawnObject(this, e->pos());
    }
}
