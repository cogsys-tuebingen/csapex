/*
 * global_option_manager.h
 *
 *  Created on: Apr 2, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef GLOBAL_OPTION_MANAGER_H
#define GLOBAL_OPTION_MANAGER_H

/// COMPONENT
#include <vision_evaluator/global_option.h>

/// PROJECT
#include <csapex/plugin_manager.hpp>

namespace csapex
{

class GlobalOptionManager : public GlobalOption, PluginManager<GlobalOption>
{
    Q_OBJECT

public:
    GlobalOptionManager();

    virtual void insert(QBoxLayout* parent);

private:
    std::vector<GlobalOption::Ptr> options;
};

}

#endif // GLOBAL_OPTION_MANAGER_H
