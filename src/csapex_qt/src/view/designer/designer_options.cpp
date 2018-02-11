/// HEADER
#include <csapex/view/designer/designer_options.h>

/// PROJECT
#include <csapex/core/settings.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/view/designer/designer.h>
#include <csapex/view/designer/designer_scene.h>
#include <csapex/view/designer/graph_view.h>
#include <csapex/view/widgets/minimap_widget.h>

using namespace csapex;


DesignerOptions::DesignerOptions(Settings &settings, Designer *designer)
    : settings_(settings), designer_(designer)
{
}

void DesignerOptions::setup(GraphView *view)
{
    DesignerScene* designer_scene = view->designerScene();

    if(settings_.knows("grid-lock")) {
        enableGridLock(settings_.get<bool>("grid-lock"));
    }
    if(settings_.knows("grid")) {
        enableGrid(settings_.get<bool>("grid"));
    }

    if(settings_.knows("schematics")) {
        enableSchematics(settings_.get<bool>("schematics"));
    }
    if(settings_.knows("debug")) {
        designer_scene->enableDebug(settings_.get<bool>("debug"));
    }

    for(const std::string& type : {"messages", "signals", "active", "inactive"}) {
        std::string key = "display-" + type;
        if(settings_.knows(key)) {
            designer_scene->displayConnections(QString::fromStdString(type), settings_.get<bool>(key));
        } else {
            designer_scene->displayConnections(QString::fromStdString(type), true);
        }
    }
}

bool DesignerOptions::isGridEnabled() const
{
    return settings_.getTemporary<bool>("grid", false);
}
bool DesignerOptions::isGridLockEnabled() const
{
    return settings_.getTemporary<bool>("grid-lock", false);
}
bool DesignerOptions::isSchematicsEnabled() const
{
    return settings_.getTemporary<bool>("schematics", false);
}

bool DesignerOptions::isGraphComponentsEnabled() const
{
    return settings_.getTemporary<bool>("display-graph-components", false);
}
bool DesignerOptions::isThreadsEnabled() const
{
    return settings_.getTemporary<bool>("display-threads", false);
}
bool DesignerOptions::isFrequencyEnabled() const
{
    return settings_.getTemporary<bool>("display-frequencies", false);
}

bool DesignerOptions::isMinimapEnabled() const
{
    return settings_.getTemporary<bool>("display-minimap", false);
}

bool DesignerOptions::isDebug() const
{
    return settings_.getTemporary<bool>("debug", false);
}


void DesignerOptions::enableGrid(bool grid)
{
    if(!settings_.knows("grid")) {
        settings_.addPersistent(csapex::param::ParameterFactory::declareBool("grid", grid));
    }

    settings_.set("grid", grid);

    for(const auto& pair : designer_->graph_views_) {
        GraphView* view = pair.second;
        view->designerScene()->enableGrid(grid);

        view->setCacheMode(QGraphicsView::CacheNone);
        view->setCacheMode(QGraphicsView::CacheBackground);
    }

    Q_EMIT gridEnabled(grid);

}
void DesignerOptions::enableGridLock(bool enabled)
{
    if(!settings_.knows("grid-lock")) {
        settings_.addPersistent(csapex::param::ParameterFactory::declareBool("grid-lock", enabled));
    }

    settings_.set("grid-lock", enabled);

    Q_EMIT gridLockEnabled(enabled);
}

void DesignerOptions::enableSchematics(bool schema)
{
    if(!settings_.knows("schematics")) {
        settings_.addPersistent(csapex::param::ParameterFactory::declareBool("schematics", schema));
    }

    settings_.set("schematics", schema);

    for(const auto& pair : designer_->graph_views_) {
        GraphView* view = pair.second;
        view->designerScene()->enableSchema(schema);
    }

    Q_EMIT schematicsEnabled(schema);

}

void DesignerOptions::displayGraphComponents(bool display)
{
    if(!settings_.knows("display-graph-components")) {
        settings_.addPersistent(csapex::param::ParameterFactory::declareBool("display-graph-components", display));
    }

    settings_.set("display-graph-components", display);

    for(const auto& pair : designer_->graph_views_) {
        GraphView* view = pair.second;
        view->updateBoxInformation();
    }

    Q_EMIT graphComponentsEnabled(display);
}

void DesignerOptions::displayThreads(bool display)
{
    if(!settings_.knows("display-threads")) {
        settings_.addPersistent(csapex::param::ParameterFactory::declareBool("display-threads", display));
    }

    settings_.set("display-threads", display);

    for(const auto& pair : designer_->graph_views_) {
        GraphView* view = pair.second;
        view->updateBoxInformation();
    }

    Q_EMIT threadsEnabled(display);
}

void DesignerOptions::displayFrequency(bool display)
{
    if(!settings_.knows("display-frequencies")) {
        settings_.addPersistent(csapex::param::ParameterFactory::declareBool("display-frequencies", display));
    }

    settings_.set("display-frequencies", display);

    for(const auto& pair : designer_->graph_views_) {
        GraphView* view = pair.second;
        view->updateBoxInformation();
    }

    Q_EMIT frequencyEnabled(display);
}


void DesignerOptions::displayMinimap(bool display)
{
    if(!settings_.knows("display-minimap")) {
        settings_.addPersistent(csapex::param::ParameterFactory::declareBool("display-minimap", display));
    }

    settings_.set("display-minimap", display);

    designer_->minimap_->setVisible(display);

    Q_EMIT minimapEnabled(display);
}
void DesignerOptions::enableDebug(bool debug)
{
    if(!settings_.knows("debug")) {
        settings_.addPersistent(csapex::param::ParameterFactory::declareBool("debug", debug));
    }

    settings_.set("debug", debug);

    for(const auto& pair : designer_->graph_views_) {
        GraphView* view = pair.second;
        view->designerScene()->enableDebug(debug);
    }

    Q_EMIT debugEnabled(debug);
}




bool DesignerOptions::areMessageConnectionsVisibile() const
{
    return settings_.getPersistent<bool>("display-messages", true);
}
bool DesignerOptions::areSignalConnectionsVisible() const
{
    return settings_.getPersistent<bool>("display-signals", true);
}
bool DesignerOptions::areActiveConnectionsVisible() const
{
    return settings_.getPersistent<bool>("display-active", true);
}
bool DesignerOptions::areInactiveConnectionsVisibile() const
{
    return settings_.getPersistent<bool>("display-inactive", true);
}

void DesignerOptions::displaySignalConnections(bool display)
{
    displayConnections("signals", display);
}
void DesignerOptions::displayMessageConnections(bool display)
{
    displayConnections("messages", display);
}
void DesignerOptions::displayActiveConnections(bool display)
{
    displayConnections("active", display);
}
void DesignerOptions::displayInactiveConnections(bool display)
{
    displayConnections("inactive", display);
}

void DesignerOptions::displayConnections(const std::string& type, bool display)
{
    QString q_type = QString::fromStdString(type);
    std::string key = std::string("display-") + type;

    if(!settings_.knows(key)) {
        settings_.addPersistent(csapex::param::ParameterFactory::declareBool(key, display));
    }

    settings_.set(key, display);

    for(const auto& pair : designer_->graph_views_) {
        GraphView* view = pair.second;
        view->designerScene()->displayConnections(q_type, display);
    }

    if(type == "messages") {
        messagesEnabled(display);
    } else if(type == "signals") {
        signalsEnabled(display);
    } else if(type == "active") {
        activeEnabled(display);
    } else if(type == "inactive") {
        inactiveEnabled(display);
    }
}


/// MOC
#include "../../../include/csapex/view/designer/moc_designer_options.cpp"
