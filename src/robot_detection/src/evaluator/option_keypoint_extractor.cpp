/// HEADER
#include "option_keypoint_extractor.h"

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <vision_evaluator/qt_helper.hpp>

PLUGINLIB_EXPORT_CLASS(robot_detection::OptionKeypointExtractor, vision_evaluator::GlobalOption)

using namespace robot_detection;


OptionKeypointExtractor::OptionKeypointExtractor()
    : selection(NULL), threshold(NULL)
{
}

OptionKeypointExtractor::~OptionKeypointExtractor()
{
}

void OptionKeypointExtractor::update_type(int slot)
{
    Config current = Config::getGlobal();
    current.setKeypointType(static_cast<Types::Keypoint::ID>(slot));
    current.replaceGlobal();
}

void OptionKeypointExtractor::update_threshold(int t)
{
    Config current = Config::getGlobal();
    current.extractor_threshold = t;
    current.replaceGlobal();
}

void OptionKeypointExtractor::insert(QBoxLayout* layout)
{
    selection = new QComboBox;
    for(int k = 0; k < Types::Keypoint::COUNT; k++) {
        selection->addItem(Types::Keypoint::write(k).c_str());
    }
    Config config = Config::getGlobal();
    layout->addLayout(QtHelper::wrap("Keypoint", selection));

    QObject::connect(selection, SIGNAL(currentIndexChanged(int)), this, SLOT(update_type(int)));


    //threshold = new QSlider(Qt::Horizontal);
    //threshold->setMinimum(1);
    //threshold->setMaximum(200);
    //threshold->setValue(config.extractor_threshold);

    threshold = QtHelper::makeSlider(layout, "threshold", config.extractor_threshold, 1, 200);

    layout->addWidget(threshold);

    QObject::connect(threshold, SIGNAL(valueChanged(int)), this, SLOT(update_threshold(int)));
}

void OptionKeypointExtractor::configChanged()
{
    if(config.getKeypointType() < 0) {
        return;
    }

    if(selection != NULL) {
        selection->setCurrentIndex(config.getKeypointType());
    }
}

