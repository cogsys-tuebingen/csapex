/// HEADER
#include <csapex/view/designer.h>

/// COMPONENT
#include <csapex/command/dispatcher.h>
#include <csapex/command/meta.h>
#include <csapex/core/drag_io.h>
#include <csapex/core/settings.h>
#include <csapex/model/node_factory.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/node.h>
#include <csapex/utility/qt_helper.hpp>
#include <csapex/view/box.h>
#include <csapex/view/designer_view.h>
#include <csapex/view/widget_controller.h>
#include "ui_designer.h"
#include <utils_param/parameter_factory.h>
#include <csapex/view/designer_scene.h>
#include <csapex/utility/movable_graphics_proxy_widget.h>
#include <csapex/view/minimap_widget.h>

/// SYSTEM
#include <QScrollBar>
#include <QGLWidget>
#include <QGraphicsView>
#include <QGraphicsSceneWheelEvent>

using namespace csapex;

Designer::Designer(Settings& settings, Graph::Ptr graph, CommandDispatcher *dispatcher, WidgetControllerPtr widget_ctrl,
                   DesignerView* view, DesignerScene* scene, MinimapWidget* minimap,
                   QWidget* parent)
    : QWidget(parent), ui(new Ui::Designer),
      designer_view_(view), designer_scene_(scene), minimap_(minimap),
      settings_(settings), graph_(graph), dispatcher_(dispatcher), widget_ctrl_(widget_ctrl), is_init_(false)
{
}

Designer::~Designer()
{
    delete ui;
}

void Designer::setup()
{
    ui->setupUi(this);

    ui->horizontalLayout->addWidget(designer_view_);
    designer_view_->setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers | QGL::DoubleBuffer)));
    designer_view_->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);


    designer_view_->setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);

    QObject::connect(designer_view_, SIGNAL(selectionChanged()), this, SIGNAL(selectionChanged()));

//    designer_view_->setSceneRect(0, 0, 1000, 1000);

    //    designer_board = board;
    //    ui->scrollArea->setWidget(designer_board);

    if(settings_.knows("grid")) {
        enableGrid(settings_.get<bool>("grid"));
    }

    if(settings_.knows("schematics")) {
        enableSchematics(settings_.get<bool>("schematics"));
    }

    setFocusPolicy(Qt::NoFocus);

//    ui->horizontalLayout->addWidget(minimap_);
    minimap_->setParent(designer_view_);
    minimap_->move(10, 10);
    minimap_->setVisible(true);
}

std::vector<NodeBox*> Designer::getSelectedBoxes() const
{
    std::vector<NodeBox*> r;
    foreach(QGraphicsItem* item, designer_scene_->selectedItems()) {
        MovableGraphicsProxyWidget* proxy = dynamic_cast<MovableGraphicsProxyWidget*>(item);
        if(proxy) {
            r.push_back(proxy->getBox());
        }
    }
    return r;
}

void Designer::clearSelection()
{
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

    if(del->commands() != 0) {
        dispatcher_->execute(del);
    }
}

void Designer::overwriteStyleSheet(QString &stylesheet)
{
    designer_view_->overwriteStyleSheet(stylesheet);
}

void Designer::setView(int /*sx*/, int /*sy*/)
{
    //designer_board->setView(sx, sy);
}

DesignerView* Designer::getDesignerView()
{
    return designer_view_;
}

void Designer::refresh()
{
    designer_scene_->invalidateSchema();
}

void Designer::reset()
{
    designer_view_->reset();
}

void Designer::addBox(NodeBox *box)
{
    QObject::connect(box, SIGNAL(helpRequest(NodeBox*)), this, SIGNAL(helpRequest(NodeBox*)));

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
bool Designer::isSchematicsEnabled() const
{
    return settings_.get<bool>("schematics", false);
}

bool Designer::isGraphComponentsEnabled() const
{
    return settings_.get<bool>("display-graph-components", false);
}
bool Designer::isThreadsEnabled() const
{
    return settings_.get<bool>("display-threads", false);
}

bool Designer::isMinimapEnabled() const
{
    return settings_.get<bool>("display-minimap", false);
}

bool Designer::hasSelection() const
{
    return designer_scene_->selectedItems().size() > 0;
}

void Designer::enableGrid(bool grid)
{
    if(!settings_.knows("grid")) {
        settings_.add(param::ParameterFactory::declareBool("grid", grid));
    }

    settings_.set("grid", grid);

    designer_scene_->enableGrid(grid);

    Q_EMIT gridEnabled(grid);

}

void Designer::enableSchematics(bool schema)
{
    if(!settings_.knows("schematics")) {
        settings_.add(param::ParameterFactory::declareBool("schematics", schema));
    }

    settings_.set("schematics", schema);

    designer_scene_->enableSchema(schema);

    Q_EMIT schematicsEnabled(schema);

}

void Designer::displayGraphComponents(bool display)
{
    if(!settings_.knows("display-graph-components")) {
        settings_.add(param::ParameterFactory::declareBool("display-graph-components", display));
    }

    settings_.set("display-graph-components", display);

    designer_view_->updateBoxInformation();

    Q_EMIT graphComponentsEnabled(display);
}

void Designer::displayThreads(bool display)
{
    if(!settings_.knows("display-threads")) {
        settings_.add(param::ParameterFactory::declareBool("display-threads", display));
    }

    settings_.set("display-threads", display);

    designer_view_->updateBoxInformation();

    Q_EMIT threadsEnabled(display);
}


void Designer::displayMinimap(bool display)
{
    if(!settings_.knows("display-minimap")) {
        settings_.add(param::ParameterFactory::declareBool("display-minimap", display));
    }

    settings_.set("display-minimap", display);

    minimap_->setVisible(display);

    Q_EMIT minimapEnabled(display);
}
