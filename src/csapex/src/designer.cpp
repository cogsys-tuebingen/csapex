/// HEADER
#include <csapex/designer.h>

/// COMPONENT
#include <csapex/box.h>
#include <csapex/command_dispatcher.h>
#include <csapex/design_board.h>
#include "ui_designer.h"

using namespace csapex;

Q_DECLARE_METATYPE(QSharedPointer<QImage>)

Designer::Designer(Graph &graph, QWidget* parent)
    : QWidget(parent), ui(new Ui::Designer)
{
    qRegisterMetaType<QSharedPointer<QImage> >("QSharedPointer<QImage>");

    ui->setupUi(this);

    designer_board = new DesignBoard(graph);
    ui->scrollArea->setWidget(designer_board);

    CommandDispatcher::instance().setGraph(&graph);
}

Designer::~Designer()
{
}

bool Designer::eventFilter(QObject* o, QEvent* e)
{
    return true;
}

void Designer::keyPressEvent(QKeyEvent* e)
{
    designer_board->keyPressEvent(e);
}

void Designer::keyReleaseEvent(QKeyEvent* e)
{
    designer_board->keyReleaseEvent(e);
}

void Designer::resizeEvent(QResizeEvent* e)
{
}

void Designer::addBox(Box *box)
{
//    box->setParent(this);

    designer_board->addBoxEvent(box);
}


void Designer::deleteBox(Box *box)
{
    designer_board->refresh();
}

void Designer::stateChangedEvent()
{
    designer_board->refresh();
}
