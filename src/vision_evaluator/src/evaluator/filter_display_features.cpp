/// HEADER
#include "filter_display_features.h"

/// PROJECT
#include <utils/extractor.h>

REGISTER_FILTER(FilterDisplayFeatures)

FilterDisplayFeatures::FilterDisplayFeatures()
    : Filter("Display Features")
{
}

Filter::TypePtr FilterDisplayFeatures::createInstance(CONSTRUCTOR_MODE mode)
{
    return Filter::TypePtr(new FilterDisplayFeatures());
}

void FilterDisplayFeatures::filter(cv::Mat img, cv::Mat mask)
{
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    tools->getExtractor()->extract(img, mask, keypoints, descriptors);

    FilterDisplayFeaturesOption* opt = get<Option, FilterDisplayFeaturesOption>();
    cv::drawKeypoints(img, keypoints, img, opt ? opt->color : cv::Scalar::all(-1), opt->flags);
}

void FilterDisplayFeatures::insert(QLayout*)
{

}

REGISTER_OPTION(FilterDisplayFeaturesOption)

FilterDisplayFeaturesOption::FilterDisplayFeaturesOption()
    : Option("Feature Color"), color(cv::Scalar::all(-1))
{

}

Option::TypePtr FilterDisplayFeaturesOption::createInstance(CONSTRUCTOR_MODE mode)
{
    return Option::TypePtr(new FilterDisplayFeaturesOption);
}

void FilterDisplayFeaturesOption::insert(QLayout* layout)
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

void FilterDisplayFeaturesOption::update()
{
    flags = 0;
    if(richbox->isChecked()) flags += cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS;

    Q_EMIT plugin_changed();
}

void FilterDisplayFeaturesOption::update(int slot)
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

    Q_EMIT plugin_changed();
}
