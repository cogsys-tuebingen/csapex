/// HEADER
#include "filter_display_features.h"

/// PROJECT
#include <utils/extractor.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(robot_detection::FilterDisplayFeatures, vision_evaluator::Filter)

using namespace robot_detection;

FilterDisplayFeatures::FilterDisplayFeatures()
    : color(cv::Scalar::all(-1))
{
}

void FilterDisplayFeatures::filter(cv::Mat &img, cv::Mat &mask)
{
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    tools->getExtractor()->extract(img, mask, keypoints, descriptors);

    cv::drawKeypoints(img, keypoints, img, color, flags);
}

void FilterDisplayFeatures::insert(QBoxLayout* layout)
{
    colorbox = new QComboBox;
    colorbox->addItem("Random Color");
    colorbox->addItem("Black");
    colorbox->addItem("White");
    colorbox->addItem("Red");
    layout->addWidget(colorbox);
    QObject::connect(colorbox, SIGNAL(currentIndexChanged(int)), this, SLOT(update(int)));

    richbox = new QCheckBox("Rich Keypoints");
    layout->addWidget(richbox);
    QObject::connect(richbox, SIGNAL(clicked()), this, SLOT(update()));
}


void FilterDisplayFeatures::update()
{
    flags = 0;
    if(richbox->isChecked()) flags += cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS;

    Q_EMIT filter_changed();
}

void FilterDisplayFeatures::update(int slot)
{
    switch(slot) {
    case 0:
        color = cv::Scalar::all(-1);
        break;
    case 1:
        color = cv::Scalar::all(0);
        break;
    case 2:
        color = cv::Scalar::all(255);
        break;
    case 3:
        color = cv::Scalar(0, 0, 255);
        break;
    }

    Q_EMIT filter_changed();
}
