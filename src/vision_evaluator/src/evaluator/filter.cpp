/// HEADER
#include "filter.h"

using namespace vision_evaluator;


Filter::Filter()
    : name_("unnamed")
{
}

Filter::~Filter()
{
}

std::string Filter::getName()
{
    return name_;
}

void Filter::setName(const std::string& name)
{
    name_ = name;
}

