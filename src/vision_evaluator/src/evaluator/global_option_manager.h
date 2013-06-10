/*
 * global_option_manager.h
 *
 *  Created on: Apr 2, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef GLOBAL_OPTION_MANAGER_H
#define GLOBAL_OPTION_MANAGER_H

/// COMPONENT
#include "global_option.h"
#include "registration.hpp"

/// PROJECT
#include <utils/plugin_manager.hpp>

#define REGISTER_GLOBAL_OPTION(class_name)\
    REGISTER_GENERIC(GlobalOptionManager, class_name)

namespace vision_evaluator
{

class GlobalOptionManager : public GlobalOption, PluginManager<GlobalOption>
{
    Q_OBJECT

public:
    GlobalOptionManager();

    virtual void insert(QBoxLayout* parent);

private:
    std::vector<GlobalOption*> options;
};

}

#endif // GLOBAL_OPTION_MANAGER_H
