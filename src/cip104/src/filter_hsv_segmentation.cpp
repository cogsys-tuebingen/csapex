/// HEADER
#include "filter_hsv_segmentation.h"

/// PROJECT
#include <vision_evaluator/qt_helper.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <boost/foreach.hpp>

PLUGINLIB_EXPORT_CLASS(cip104::HSVSegmentation, vision_evaluator::Filter)

using namespace cip104;

HSVSegmentation::HSVSegmentation()
    : min_h(NULL), min(cv::Scalar::all(0)), max(cv::Scalar::all(255))
{
}

void HSVSegmentation::filter(cv::Mat& img, cv::Mat& mask)
{
    cv::Mat hsv;
    cv::cvtColor(img, hsv, CV_BGR2HSV);

    cv::Mat bw;
    cv::inRange(hsv, min, max, bw);

    if(mask.empty()) {
        mask = bw;
    } else {
        cv::min(mask, bw, mask);
    }
}

void HSVSegmentation::update()
{
    min = cv::Scalar(min_h->value(), min_s->value(), min_v->value());
    max = cv::Scalar(max_h->value(), max_s->value(), max_v->value());
}

void HSVSegmentation::insert(QBoxLayout* layout)
{
    min_h = QtHelper::makeSlider(layout, "hue min", min[0], 0,255);
    max_h = QtHelper::makeSlider(layout, "hue max", max[0], 0,255);
    min_s = QtHelper::makeSlider(layout, "sat min", min[1], 0,255);
    max_s = QtHelper::makeSlider(layout, "sat max", max[1], 0,255);
    min_v = QtHelper::makeSlider(layout, "val min", min[2], 0,255);
    max_v = QtHelper::makeSlider(layout, "val max", max[2], 0,255);

    connect(min_h, SIGNAL(valueChanged(int)), this, SLOT(update()));
    connect(min_s, SIGNAL(valueChanged(int)), this, SLOT(update()));
    connect(min_v, SIGNAL(valueChanged(int)), this, SLOT(update()));
    connect(max_h, SIGNAL(valueChanged(int)), this, SLOT(update()));
    connect(max_s, SIGNAL(valueChanged(int)), this, SLOT(update()));
    connect(max_v, SIGNAL(valueChanged(int)), this, SLOT(update()));
}
