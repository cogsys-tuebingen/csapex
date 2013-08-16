/*
 * global_option.cpp
 *
 *  Created on: Mar 28, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// HEADER
#include <csapex/global_option.h>

using namespace csapex;

GlobalOption::GlobalOption()
{
}

void GlobalOption::setName(const std::string& name)
{
    name_ = name;
}

Memento::Ptr GlobalOption::getState() const
{
    return Memento::Ptr((Memento*) NULL);
}

void GlobalOption::setState(Memento::Ptr)
{

}
