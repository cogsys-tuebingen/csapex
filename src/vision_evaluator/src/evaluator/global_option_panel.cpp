/// HEADER
#include "global_option_panel.h"

/// COMPONENT
#include "registration.hpp"

/// PROJECT
#include <utils/plugin_manager.hpp>

/// SYSTEM
#include <boost/foreach.hpp>

STATIC_INIT(GlobalOptionPanel, generic, {
    SelectorProxy::ProxyConstructor c; \
    c.setType("Global Options Panel"); \
    c.setConstructor(boost::lambda::bind(boost::lambda::new_ptr<SelectorProxyImp<GlobalOptionPanel> >(), \
    boost::lambda::_1, (QWidget*) NULL)); \
    SelectorProxy::registerProxy(c); \
});

using namespace vision_evaluator;


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
        PluginManager<GlobalOption> manager("vision_evaluator::GlobalOption");
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
