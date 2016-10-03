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

    if(settings_.knows("display-messages")) {
        designer_scene->displayMessages(settings_.get<bool>("display-messages"));
    }

    if(settings_.knows("display-signals")) {
        designer_scene->displaySignals(settings_.get<bool>("display-signals"));
    }

    if(settings_.knows("debug")) {
        designer_scene->enableDebug(settings_.get<bool>("debug"));
    }
}

bool DesignerOptions::isGridEnabled() const
{
    return settings_.get<bool>("grid", false);
}
bool DesignerOptions::isGridLockEnabled() const
{
    return settings_.get<bool>("grid-lock", false);
}
bool DesignerOptions::isSchematicsEnabled() const
{
    return settings_.get<bool>("schematics", false);
}

bool DesignerOptions::isGraphComponentsEnabled() const
{
    return settings_.get<bool>("display-graph-components", false);
}
bool DesignerOptions::isThreadsEnabled() const
{
    return settings_.get<bool>("display-threads", false);
}
bool DesignerOptions::isFrequencyEnabled() const
{
    return settings_.get<bool>("display-frequencies", false);
}

bool DesignerOptions::isMinimapEnabled() const
{
    return settings_.get<bool>("display-minimap", false);
}

bool DesignerOptions::areMessageConnectionsVisibile() const
{
    return settings_.get<bool>("display-messages", true);
}

bool DesignerOptions::areSignalConnectionsVisible() const
{
    return settings_.get<bool>("display-signals", true);
}
bool DesignerOptions::isDebug() const
{
    return settings_.get<bool>("debug", false);
}


void DesignerOptions::enableGrid(bool grid)
{
    if(!settings_.knows("grid")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("grid", grid));
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
        settings_.add(csapex::param::ParameterFactory::declareBool("grid-lock", enabled));
    }

    settings_.set("grid-lock", enabled);

    Q_EMIT gridLockEnabled(enabled);
}

void DesignerOptions::enableSchematics(bool schema)
{
    if(!settings_.knows("schematics")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("schematics", schema));
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
        settings_.add(csapex::param::ParameterFactory::declareBool("display-graph-components", display));
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
        settings_.add(csapex::param::ParameterFactory::declareBool("display-threads", display));
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
        settings_.add(csapex::param::ParameterFactory::declareBool("display-frequencies", display));
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
        settings_.add(csapex::param::ParameterFactory::declareBool("display-minimap", display));
    }

    settings_.set("display-minimap", display);

    designer_->minimap_->setVisible(display);

    Q_EMIT minimapEnabled(display);
}

void DesignerOptions::displaySignalConnections(bool display)
{
    if(!settings_.knows("display-signals")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("display-signals", display));
    }

    settings_.set("display-signals", display);

    for(const auto& pair : designer_->graph_views_) {
        GraphView* view = pair.second;
        view->designerScene()->displaySignals(display);
    }


    Q_EMIT signalsEnabled(display);
}

void DesignerOptions::displayMessageConnections(bool display)
{
    if(!settings_.knows("display-messages")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("display-messages", display));
    }

    settings_.set("display-messages", display);

    for(const auto& pair : designer_->graph_views_) {
        GraphView* view = pair.second;
        view->designerScene()->displayMessages(display);
    }

    Q_EMIT messagesEnabled(display);
}

void DesignerOptions::enableDebug(bool debug)
{
    if(!settings_.knows("debug")) {
        settings_.add(csapex::param::ParameterFactory::declareBool("debug", debug));
    }

    settings_.set("debug", debug);

    for(const auto& pair : designer_->graph_views_) {
        GraphView* view = pair.second;
        view->designerScene()->enableDebug(debug);
    }

    Q_EMIT debugEnabled(debug);
}

/// MOC
#include "../../../include/csapex/view/designer/moc_designer_options.cpp"
