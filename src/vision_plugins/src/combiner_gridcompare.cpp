#include "combiner_gridcompare.h"

/// PROJECT
#include <vision_evaluator/qt_helper.hpp>

using namespace vision_evaluator;

GridCompare::GridCompare()
{
}

void GridCompare::fill(QBoxLayout *layout)
{
    ImageCombiner::fill(layout);
    slide_width_  = QtHelper::makeSlider(layout, "Grid Width",  64, 1, 640);
    slide_height_ = QtHelper::makeSlider(layout, "Grid Height", 48, 1, 640);


}
