/// HEADER
#include <csapex/view/designer.h>

/// COMPONENT
#include <csapex/command/dispatcher.h>
#include <csapex/core/drag_io.h>
#include <csapex/core/settings.h>
#include <csapex/manager/box_manager.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/node.h>
#include <csapex/utility/qt_helper.hpp>
#include <csapex/view/box.h>
#include <csapex/view/design_board.h>
#include <csapex/view/widget_controller.h>
#include "ui_designer.h"
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <QScrollBar>

using namespace csapex;

Designer::Designer(Settings& settings, Graph::Ptr graph, CommandDispatcher *dispatcher, WidgetControllerPtr widget_ctrl, DesignBoard* board, QWidget* parent)
    : QWidget(parent), ui(new Ui::Designer), settings_(settings), graph_(graph), dispatcher_(dispatcher), widget_ctrl_(widget_ctrl), is_init_(false)
{
    ui->setupUi(this);

    designer_board = board;
    ui->scrollArea->setWidget(designer_board);

    if(settings_.knows("grid")) {
        enableGrid(settings_.get<bool>("grid"));
    }

    if(settings_.knows("grid-lock")) {
        lockToGrid(settings_.get<bool>("grid-lock"));
    }
}

Designer::~Designer()
{
}

void Designer::deleteSelected()
{
    command::Meta::Ptr del(new command::Meta("delete selected"));

    if(widget_ctrl_->box_selection_.countSelected() != 0) {
        del->add(widget_ctrl_->box_selection_.deleteSelectedCommand());
    }
    if(widget_ctrl_->connection_selection_.countSelected() != 0) {
        del->add(widget_ctrl_->connection_selection_.deleteSelectedCommand());
    }

    if(del->commands() != 0) {
        dispatcher_->execute(del);
    }
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

void Designer::setView(int sx, int sy)
{
    designer_board->setView(sx, sy);
}

void Designer::reset()
{
    designer_board->reset();
}

void Designer::addBox(Box *box)
{
    designer_board->addBoxEvent(box);
}

void Designer::removeBox(Box *box)
{
    designer_board->removeBoxEvent(box);
}


void Designer::stateChangedEvent()
{
    designer_board->refresh();
}


bool Designer::isGridEnabled() const
{
    return settings_.get<bool>("grid", false);
}

bool Designer::isGridLockEnabled() const
{
    return settings_.get<bool>("grid-lock", false);
}

void Designer::enableGrid(bool grid)
{
    if(!settings_.knows("grid")) {
        settings_.add(param::ParameterFactory::declareBool("grid", grid));
    }

    settings_.set("grid", grid);

    designer_board->enableGrid(grid);
    Q_EMIT gridEnabled(grid);

}

void Designer::lockToGrid(bool lock)
{
    if(!settings_.knows("grid-lock")) {
        settings_.add(param::ParameterFactory::declareBool("grid-lock", lock));
    }

    settings_.set("grid-lock", lock);

    DragIO::lock = lock;
    Q_EMIT gridLockEnabled(lock);
}
