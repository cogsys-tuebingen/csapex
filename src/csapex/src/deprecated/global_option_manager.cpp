/*
 * global_option_manager.cpp
 *
 *  Created on: Apr 2, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// HEADER
#include <csapex/deprecated/global_option_manager.h>

/// SYSTEM
#include <boost/foreach.hpp>

using namespace csapex;

GlobalOptionManager::GlobalOptionManager()
    : manager("csapex::GlobalOption")
{
}

void GlobalOptionManager::insert(QBoxLayout* parent)
{
    if(options.empty()) {
        if(!manager.pluginsLoaded()) {
            manager.reload();
        }

        typedef const std::pair<std::string, DefaultConstructor<GlobalOption> > PAIR;
        BOOST_FOREACH(PAIR& pair, manager.availableClasses()) {
            options.push_back(pair.second.construct());
        }
    }

    BOOST_FOREACH(GlobalOption::Ptr it, options) {
        it->insert(parent);
    }

}
