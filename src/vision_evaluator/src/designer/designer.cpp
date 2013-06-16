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
#include <iostream>

using namespace vision_evaluator;

Designer::Designer(QWidget* parent)
    : QWidget(parent), ui(new Ui::Designer)
{
    ui->setupUi(this);

    BoxManager::instance().fill(ui->widget_selection->layout());

    QObject::connect(&BoxManager::instance(), SIGNAL(stateChanged()), this, SIGNAL(stateChanged()));
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
