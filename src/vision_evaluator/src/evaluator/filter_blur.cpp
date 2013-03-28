/// HEADER
#include "filter_blur.h"

/// COMPONENT
#include "filter_manager.h"

REGISTER_FILTER(FilterBlur)

using namespace vision_evaluator;

FilterBlur::FilterBlur()
    : blur(0)
{
}

void FilterBlur::filter(cv::Mat img, cv::Mat mask)
{
    if(blur > 0) {
        cv::blur(img, img, cv::Size(blur, blur));
    }
}

void FilterBlur::insert(QBoxLayout* layout)
{
    slider = makeSlider(layout, "blur", 0, 0, 26);

    QObject::connect(slider, SIGNAL(valueChanged(int)), this, SLOT(update(int)));
}


void FilterBlur::update(int slot)
{
    blur = slot;

    Q_EMIT filter_changed();
}
