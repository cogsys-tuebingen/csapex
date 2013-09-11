/*
 * global_option_manager.h
 *
 *  Created on: Apr 2, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef GLOBAL_OPTION_MANAGER_H
#define GLOBAL_OPTION_MANAGER_H

//#warning This file should not be used anymore!

/// COMPONENT
#include <csapex/deprecated/global_option.h>

/// PROJECT
#include <utils_plugin/plugin_manager.hpp>

namespace csapex
{

class GlobalOptionManager : public GlobalOption
{
    Q_OBJECT

public:
    GlobalOptionManager();

    virtual void insert(QBoxLayout* parent);

private:
    std::vector<GlobalOption::Ptr> options;

    PluginManager<GlobalOption> manager;
};

}

#endif // GLOBAL_OPTION_MANAGER_H
