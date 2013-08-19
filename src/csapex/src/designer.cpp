/// HEADER
#include <csapex/designer.h>

/// COMPONENT
#include <csapex/box.h>
#include <csapex/command_dispatcher.h>
#include <csapex/design_board.h>
#include <csapex/box_manager.h>
#include "ui_designer.h"

using namespace csapex;

Q_DECLARE_METATYPE(QSharedPointer<QImage>)

Designer::Designer(QWidget* parent)
    : QWidget(parent), ui(new Ui::Designer), menu(NULL)
{
    qRegisterMetaType<QSharedPointer<QImage> >("QSharedPointer<QImage>");

    ui->setupUi(this);

    designer_board = new DesignBoard;
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
    //    box->setParent(this);

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
