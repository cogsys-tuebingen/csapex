/// HEADER
#include "designer.h"

/// PROJECT
#include "ui_designer.h"
#include "connector.h"
#include "connector_in.h"
#include "connector_out.h"
#include "selector_proxy.h"
#include "box_manager.h"
#include "box.h"
#include "../qt_helper.hpp"
#include "designerio.h"

/// SYSTEM
#include <boost/foreach.hpp>
#include <QResizeEvent>
#include <QMenu>
#include <QScrollBar>

using namespace vision_evaluator;

Designer::Designer(QWidget* parent)
    : QWidget(parent), ui(new Ui::Designer)
{
    ui->setupUi(this);

    BoxManager::instance().fill(ui->widget_selection->layout());

    QObject::connect(&BoxManager::instance(), SIGNAL(stateChanged()), this, SIGNAL(stateChanged()));
}

bool Designer::eventFilter(QObject* o, QEvent* e)
{
    return true;
}

void Designer::keyPressEvent(QKeyEvent* e)
{
    ui->designer->keyPressEvent(e);
}

void Designer::keyReleaseEvent(QKeyEvent* e)
{
    ui->designer->keyReleaseEvent(e);
}

void Designer::resizeEvent(QResizeEvent* e)
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
    DesignerIO::save(this);
}

void Designer::load()
{
    DesignerIO::load(this);
}

void Designer::undo()
{
    BoxManager::instance().undo();
}

void Designer::redo()
{
    BoxManager::instance().redo();
}

void Designer::clear()
{
    QList<vision_evaluator::Box*> boxes = findChildren<vision_evaluator::Box*> ();
    BOOST_FOREACH(vision_evaluator::Box* box, boxes) {
        box->stop();
    }
    ui->designer->getOverlay()->clear();
    BOOST_FOREACH(vision_evaluator::Box* box, boxes) {
        delete box;
    }
}
