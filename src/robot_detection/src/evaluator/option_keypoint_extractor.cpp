/// HEADER
#include "option_keypoint_extractor.h"

/// PROJECT
#include <utils/extractor_manager.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <vision_evaluator/qt_helper.hpp>

PLUGINLIB_EXPORT_CLASS(robot_detection::OptionKeypointExtractor, vision_evaluator::GlobalOption)

using namespace robot_detection;
using namespace vision_evaluator;


OptionKeypointExtractor::OptionKeypointExtractor()
    : selection(NULL), threshold(NULL)
{
}

OptionKeypointExtractor::~OptionKeypointExtractor()
{
}

void OptionKeypointExtractor::update_type(int slot)
{
    Config config = Config::getGlobal();

    QString target = config.getKeypointType().c_str();
    if(target == selection->itemText(slot)) {
        return;
    }

    config.setKeypointType(selection->itemText(slot).toUtf8().constData());
    config.replaceGlobal();
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

    ExtractorManager manager;
    typedef std::pair<std::string, ExtractorManager::ExtractorInitializer> Pair;
    BOOST_FOREACH(Pair fc, manager.featureDetectors()) {
        selection->addItem(fc.second.getType().c_str());
    }
    layout->addLayout(QtHelper::wrap("Keypoint", selection));

    Config config = Config::getGlobal();
    QString target = config.getKeypointType().c_str();

    if(selection != NULL) {
        for(int i = 0; i < selection->count(); ++i) {
            if(selection->itemText(i) == target) {
                selection->setCurrentIndex(i);
            }
        }
    }


    QObject::connect(selection, SIGNAL(currentIndexChanged(int)), this, SLOT(update_type(int)));

    threshold = QtHelper::makeSlider(layout, "threshold", config.extractor_threshold, 1, 200);
    layout->addWidget(threshold);

    QObject::connect(threshold, SIGNAL(valueChanged(int)), this, SLOT(update_threshold(int)));
}

void OptionKeypointExtractor::configChanged()
{
//    if(config.getKeypointType() < 0) {
//        return;
//    }

    QString target = config.getKeypointType().c_str();

    if(selection != NULL) {
        for(int i = 0; i < selection->count(); ++i) {
            if(selection->itemText(i) == target) {
                selection->setCurrentIndex(i);
                return;
            }
        }
    }
}

