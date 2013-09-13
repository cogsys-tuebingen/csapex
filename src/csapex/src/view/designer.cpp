/// HEADER
#include <csapex/view/designer.h>

/// COMPONENT
#include <csapex/model/box.h>
#include <csapex/command/dispatcher.h>
#include <csapex/view/design_board.h>
#include <csapex/manager/box_manager.h>
#include "ui_designer.h"

using namespace csapex;

Designer::Designer(CommandDispatcher *dispatcher, QWidget* parent)
    : QWidget(parent), ui(new Ui::Designer), menu(NULL)
{
    ui->setupUi(this);

    designer_board = new DesignBoard(dispatcher);
    ui->scrollArea->setWidget(designer_board);
}

Designer::~Designer()
{
}

bool Designer::eventFilter(QObject*, QEvent*)
{
    return false;
}

void Designer::keyPressEvent(QKeyEvent* e)
{
    designer_board->keyPressEvent(e);
}

void Designer::keyReleaseEvent(QKeyEvent* e)
{
    designer_board->keyReleaseEvent(e);
}

void Designer::resizeEvent(QResizeEvent*)
{
    if(menu == NULL) {

        menu = new QTreeWidget;

        QWidget* boxes = ui->page;
        boxes->setLayout(new QVBoxLayout);
        BoxManager::instance().insertAvailableBoxedObjects(menu);

        boxes->layout()->addWidget(menu);
    }
}

void Designer::addBox(Box *box)
{
    designer_board->addBoxEvent(box);
}


void Designer::deleteBox(Box *)
{
    designer_board->refresh();
}

void Designer::stateChangedEvent()
{
    designer_board->refresh();
}

void Designer::enableGrid(bool grid)
{
    designer_board->enableGrid(grid);
}
