/// HEADER
#include "box_manager.h"

/// COMPONENT
#include "selector_proxy.h"

/// SYSTEM
#include <boost/foreach.hpp>

using namespace vision_evaluator;

BoxManager::BoxManager()
    : available_elements("vision_evaluator::SelectorProxy")
{
}

void BoxManager::fill(QLayout* layout)
{
    typedef std::pair<std::string, SelectorProxy::ProxyConstructor> Pair;
    BOOST_FOREACH(Pair p, available_elements.availableClasses()) {
        layout->addWidget(p.second());
    }
}

void BoxManager::register_box_type(SelectorProxy::ProxyConstructor provider)
{
    available_elements.registerConstructor(provider);
}
