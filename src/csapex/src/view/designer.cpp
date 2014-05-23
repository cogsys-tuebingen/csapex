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
#include <csapex/view/designer_view.h>
#include <csapex/view/widget_controller.h>
#include "ui_designer.h"
#include <utils_param/parameter_factory.h>
#include <csapex/view/designer_scene.h>

/// SYSTEM
#include <QScrollBar>
#include <QGLWidget>
#include <QGraphicsView>
#include <QGraphicsSceneWheelEvent>

using namespace csapex;

Designer::Designer(Settings& settings, Graph::Ptr graph, CommandDispatcher *dispatcher, WidgetControllerPtr widget_ctrl, DesignerView* view, QWidget* parent)
    : QWidget(parent), ui(new Ui::Designer), designer_view_(view), settings_(settings), graph_(graph), dispatcher_(dispatcher), widget_ctrl_(widget_ctrl), is_init_(false)
{
}

Designer::~Designer()
{
}

void Designer::setup()
{
    ui->setupUi(this);

    ui->horizontalLayout->addWidget(designer_view_);
    designer_view_->setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
    designer_view_->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);


    designer_view_->setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);

    QObject::connect(designer_view_, SIGNAL(selectionChanged()), this, SIGNAL(selectionChanged()));

//    designer_view_->setSceneRect(0, 0, 1000, 1000);

    //    designer_board = board;
    //    ui->scrollArea->setWidget(designer_board);

    if(settings_.knows("grid")) {
        enableGrid(settings_.get<bool>("grid"));
    }

    if(settings_.knows("grid-lock")) {
        lockToGrid(settings_.get<bool>("grid-lock"));
    }

    setFocusPolicy(Qt::NoFocus);
}

void Designer::clearSelection()
{
    widget_ctrl_->connection_selection_.clearSelection();

    designer_view_->scene()->clearSelection();
}

void Designer::selectAll()
{
    designer_view_->selectAll();
}

void Designer::deleteSelected()
{
    command::Meta::Ptr del(new command::Meta("delete selected"));

    del->add(designer_view_->deleteSelected());

    if(widget_ctrl_->connection_selection_.countSelected() != 0) {
        del->add(widget_ctrl_->connection_selection_.deleteSelectedCommand());
    }

    if(del->commands() != 0) {
        dispatcher_->execute(del);
    }
}

void Designer::overwriteStyleSheet(QString &stylesheet)
{
    designer_view_->overwriteStyleSheet(stylesheet);
}

void Designer::setView(int sx, int sy)
{
    //designer_board->setView(sx, sy);
}

void Designer::reset()
{
    designer_view_->reset();
}

void Designer::addBox(NodeBox *box)
{
    designer_view_->addBoxEvent(box);
}

void Designer::removeBox(NodeBox *box)
{
    designer_view_->removeBoxEvent(box);
}


void Designer::stateChangedEvent()
{
    //designer_board->refresh();
}


bool Designer::isGridEnabled() const
{
    return settings_.get<bool>("grid", false);
}

bool Designer::isGridLockEnabled() const
{
    return settings_.get<bool>("grid-lock", false);
}

bool Designer::hasSelection() const
{
    return designer_view_->hasSelection();
}

void Designer::enableGrid(bool grid)
{
    if(!settings_.knows("grid")) {
        settings_.add(param::ParameterFactory::declareBool("grid", grid));
    }

    settings_.set("grid", grid);

    designer_view_->enableGrid(grid);

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
