/*
 * global_option.cpp
 *
 *  Created on: Mar 28, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// HEADER
#include "global_option.h"

using namespace vision_evaluator;

GlobalOption::GlobalOption()
{
}

void GlobalOption::setName(const std::string& name)
{
    name_ = name;
}
