/*
 * global_option_manager.cpp
 *
 *  Created on: Apr 2, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// HEADER
#include "global_option_manager.h"

using namespace vision_evaluator;

GlobalOptionManager::GlobalOptionManager()
    : GenericManager<vision_evaluator::GlobalOption>("vision_evaluator::GlobalOption")
{
}

void GlobalOptionManager::insert(QBoxLayout *parent)
{
    if(options.empty()) {
        if(!plugins_loaded_){
            reload();
        }

        for(std::vector<Constructor>::iterator it = available_classes.begin(); it != available_classes.end(); ++it) {
            options.push_back(it->constructor());
        }
    }

    for(std::vector<GlobalOption*>::iterator it = options.begin(); it != options.end(); ++it) {
        (*it)->insert(parent);
    }

}
