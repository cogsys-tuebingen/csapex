/// HEADER
#include "global_option_panel.h"

/// PROJECT
#include <utils_plugin/plugin_manager.hpp>

/// SYSTEM
#include <boost/foreach.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::GlobalOptionPanel, csapex::BoxedObject)

using namespace csapex;


GlobalOptionPanel::State::State(GlobalOptionPanel* parent)
    : parent(parent)
{
}

void GlobalOptionPanel::State::readYaml(const YAML::Node &node)
{
    parent->ensureLoaded();

    BOOST_FOREACH(GlobalOption::Ptr it, parent->options) {
        Memento::Ptr state = it->getState();
        state->readYaml(node);
        it->setState(state);
    }
}

void GlobalOptionPanel::State::writeYaml(YAML::Emitter &out) const
{
    BOOST_FOREACH(Memento::Ptr memento, states) {
        memento->writeYaml(out);
    }
}

GlobalOptionPanel::GlobalOptionPanel()
    : state(this)
{
    setIcon(QIcon(":/wrench.png"));
}

void GlobalOptionPanel::messageArrived(ConnectorIn *source)
{
    // NO INPUTS
}

GlobalOptionPanel::~GlobalOptionPanel()
{

}

void GlobalOptionPanel::ensureLoaded()
{
    if(options.empty()) {
        PluginManager<GlobalOption> manager("csapex::GlobalOption");
        if(!manager.pluginsLoaded()) {
            manager.reload();
        }

        typedef const std::pair<std::string, PluginManager<GlobalOption>::Constructor> PAIR;
        BOOST_FOREACH(PAIR& pair, manager.availableClasses()) {
            options.push_back(pair.second.construct());
        }
    }
}

void GlobalOptionPanel::fill(QBoxLayout* layout)
{
    ensureLoaded();

    BOOST_FOREACH(GlobalOption::Ptr it, options) {
        it->insert(layout);
    }
}

bool GlobalOptionPanel::canBeDisabled() const
{
    return false;
}

Memento::Ptr GlobalOptionPanel::getState() const
{
    State::Ptr memento(new State((GlobalOptionPanel*) this));
    *memento = state;

    memento->states.clear();
    BOOST_FOREACH(GlobalOption::Ptr it, options) {
        Memento::Ptr mem = it->getState();

        if(mem.get()) {
            memento->states.push_back(mem);
        }
    }

    return memento;
}

void GlobalOptionPanel::setState(Memento::Ptr memento)
{
    State::Ptr m = boost::dynamic_pointer_cast<State> (memento);
    assert(m.get());

    state = *m;
}
