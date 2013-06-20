/// HEADER
#include "designer.h"

/// PROJECT
#include "ui_designer.h"
#include "connector.h"
#include "selector_proxy.h"
#include "box_manager.h"
#include "box.h"

/// SYSTEM
#include <boost/foreach.hpp>
#include <QResizeEvent>
#include <QMenu>
#include <QScrollBar>
#include <iostream>

using namespace vision_evaluator;

Designer::Designer(QWidget* parent)
    : QWidget(parent), ui(new Ui::Designer)
{
    ui->setupUi(this);

    BoxManager::instance().fill(ui->widget_selection->layout());

//    ui->scrollArea->installEventFilter(this);

    QObject::connect(&BoxManager::instance(), SIGNAL(stateChanged()), this, SIGNAL(stateChanged()));
}

bool Designer::eventFilter(QObject *o, QEvent *e)
{
//    if(e->type() == QEvent::Resize) {
//        QSize old = (dynamic_cast<QResizeEvent*> (e))->size();

//        int left,top,right,bottom;
//        ui->scrollArea->getContentsMargins(&left, &top, &right, &bottom);
//        QSize scrollsize(ui->scrollArea->verticalScrollBar()->width()+left+right, ui->scrollArea->horizontalScrollBar()->height()+top+bottom);

//        QResizeEvent ev (old - scrollsize,old);

//        ui->designer->resizeEvent(&ev);
//    }

    return true;
}

void Designer::keyPressEvent(QKeyEvent *e)
{
    ui->designer->keyPressEvent(e);
}

void Designer::keyReleaseEvent(QKeyEvent *e)
{
    ui->designer->keyReleaseEvent(e);
}

void Designer::resizeEvent(QResizeEvent *e)
{
//    ui->designer->resizeEvent(e);
}

bool Designer::isDirty()
{
    return BoxManager::instance().isDirty();
}


bool Designer::canUndo()
{
    return BoxManager::instance().canUndo();
}


bool Designer::canRedo()
{
    return BoxManager::instance().canRedo();
}

void Designer::save()
{
    QList<vision_evaluator::Box*> boxes = findChildren<vision_evaluator::Box*> ();
    BOOST_FOREACH(vision_evaluator::Box* box, boxes) {

    }
}
void Designer::load()
{

}
void Designer::undo()
{
    BoxManager::instance().undo();
}
void Designer::redo()
{
    BoxManager::instance().redo();
}
