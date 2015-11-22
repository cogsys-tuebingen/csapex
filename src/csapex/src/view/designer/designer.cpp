/// HEADER
#include <csapex/view/designer/designer.h>

/// COMPONENT
#include <csapex/command/dispatcher.h>
#include <csapex/command/meta.h>
#include <csapex/core/settings.h>
#include <csapex/factory/node_factory.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/node.h>
#include <csapex/view/utility/qt_helper.hpp>
#include <csapex/view/node/box.h>
#include <csapex/view/designer/designer_view.h>
#include <csapex/view/designer/widget_controller.h>
#include "ui_designer.h"
#include <csapex/param/parameter_factory.h>
#include <csapex/view/designer/designer_scene.h>
#include <csapex/view/widgets/minimap_widget.h>

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

    QObject::connect(designer_view_, SIGNAL(selectionChanged()), this, SIGNAL(selectionChanged()));

    if(settings_.knows("grid")) {
        enableGrid(settings_.get<bool>("grid"));
    }

    if(settings_.knows("schematics")) {
        enableSchematics(settings_.get<bool>("schematics"));
    }

    if(settings_.knows("display-messages")) {
        designer_scene_->displayMessages(settings_.get<bool>("display-messages"));
    }

    if(settings_.knows("display-signals")) {
        designer_scene_->displaySignals(settings_.get<bool>("display-signals"));
    }

    if(settings_.knows("debug")) {
        designer_scene_->enableDebug(settings_.get<bool>("debug"));
    }
    setFocusPolicy(Qt::NoFocus);

//    ui->horizontalLayout->addWidget(minimap_);
    minimap_->setParent(designer_view_);
    minimap_->move(10, 10);
}

std::vector<NodeBox*> Designer::getSelectedBoxes() const
{
    return designer_scene_->getSelectedBoxes();
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

    minimap_->update();
}

void Designer::removeBox(NodeBox *box)
{
    designer_view_->removeBoxEvent(box);

    minimap_->update();
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

bool Designer::areMessageConnectionsVisibile() const
{
    return settings_.get<bool>("display-messages", true);
}

bool Designer::areSignalConnectionsVisible() const
{
    return settings_.get<bool>("display-signals", true);
}
bool Designer::isDebug() const
{
    return settings_.get<bool>("debug", true);
}

bool Designer::hasSelection() const
{
    return designer_scene_->selectedItems().size() > 0;
}

void Designer::enableGrid(bool grid)
{
    if(!settings_.knows("grid")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("grid", grid));
    }

    settings_.set("grid", grid);

    designer_scene_->enableGrid(grid);

    designer_view_->setCacheMode(QGraphicsView::CacheNone);
    designer_view_->setCacheMode(QGraphicsView::CacheBackground);

    Q_EMIT gridEnabled(grid);

}

void Designer::enableSchematics(bool schema)
{
    if(!settings_.knows("schematics")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("schematics", schema));
    }

    settings_.set("schematics", schema);

    designer_scene_->enableSchema(schema);

    Q_EMIT schematicsEnabled(schema);

}

void Designer::displayGraphComponents(bool display)
{
    if(!settings_.knows("display-graph-components")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("display-graph-components", display));
    }

    settings_.set("display-graph-components", display);

    designer_view_->updateBoxInformation();

    Q_EMIT graphComponentsEnabled(display);
}

void Designer::displayThreads(bool display)
{
    if(!settings_.knows("display-threads")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("display-threads", display));
    }

    settings_.set("display-threads", display);

    designer_view_->updateBoxInformation();

    Q_EMIT threadsEnabled(display);
}


void Designer::displayMinimap(bool display)
{
    if(!settings_.knows("display-minimap")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("display-minimap", display));
    }

    settings_.set("display-minimap", display);

    minimap_->setVisible(display);

    Q_EMIT minimapEnabled(display);
}

void Designer::displaySignalConnections(bool display)
{
    if(!settings_.knows("display-signals")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("display-signals", display));
    }

    settings_.set("display-signals", display);

    designer_scene_->displaySignals(display);


    Q_EMIT signalsEnabled(display);
}

void Designer::displayMessageConnections(bool display)
{
    if(!settings_.knows("display-messages")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("display-messages", display));
    }

    settings_.set("display-messages", display);

    designer_scene_->displayMessages(display);

    Q_EMIT messagesEnabled(display);
}

void Designer::enableDebug(bool debug)
{
    if(!settings_.knows("debug")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("debug", debug));
    }

    settings_.set("debug", debug);

    designer_scene_->enableDebug(debug);

    Q_EMIT debugEnabled(debug);
}
/// MOC
#include "../../../include/csapex/view/designer/moc_designer.cpp"
