/// HEADER
#include "image_combiner.h"

using namespace vision_evaluator;

void ImageCombiner::setName(const std::string& name)
{
    name_ = name;
}

const std::string& ImageCombiner::getName()
{
    return name_;
}

void ImageCombiner::insert(QBoxLayout* layout)
{

}
