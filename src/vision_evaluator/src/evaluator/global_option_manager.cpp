/*
 * global_option_manager.cpp
 *
 *  Created on: Apr 2, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// HEADER
#include "global_option_manager.h"

/// SYSTEM
#include <boost/foreach.hpp>

using namespace csapex;

GlobalOptionManager::GlobalOptionManager()
    : PluginManager<csapex::GlobalOption>("csapex::GlobalOption")
{
}

void GlobalOptionManager::insert(QBoxLayout* parent)
{
    if(options.empty()) {
        if(!pluginsLoaded()) {
            reload();
        }

        typedef const std::pair<std::string, Constructor> PAIR;
        BOOST_FOREACH(PAIR& pair, availableClasses()) {
            options.push_back(pair.second.construct());
        }
    }

    BOOST_FOREACH(GlobalOption::Ptr it, options) {
        it->insert(parent);
    }

}
