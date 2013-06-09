/// HEADER
#include "option_keypoint_descriptor.h"

/// PROJECT
#include <utils/extractor_manager.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <vision_evaluator/qt_helper.hpp>

PLUGINLIB_EXPORT_CLASS(robot_detection::OptionKeypointDescriptor, vision_evaluator::GlobalOption)

using namespace robot_detection;
using namespace vision_evaluator;


OptionKeypointDescriptor::OptionKeypointDescriptor()
    : selection(NULL)
{
}

OptionKeypointDescriptor::~OptionKeypointDescriptor()
{
}

void OptionKeypointDescriptor::update(int slot)
{
    Config config = Config::getGlobal();

    QString target = config.getDescriptorType().c_str();
    if(target == selection->itemText(slot)) {
        return;
    }

    std::cout << slot << std::endl;

    config.setDescriptorType(selection->itemText(slot).toUtf8().constData());
    config.replaceGlobal();
}

void OptionKeypointDescriptor::insert(QBoxLayout* layout)
{
    selection = new QComboBox;

    ExtractorManager manager;
    typedef std::pair<std::string, ExtractorManager::ExtractorInitializer> Pair;
    BOOST_FOREACH(Pair fc, manager.descriptorExtractors()) {
        selection->addItem(fc.second.getName().c_str());
    }
    layout->addLayout(QtHelper::wrap("Descriptor", selection));

    Config config = Config::getGlobal();
    QString target = config.getDescriptorType().c_str();

    if(selection != NULL) {
        for(int i = 0; i < selection->count(); ++i) {
            if(selection->itemText(i) == target) {
                selection->setCurrentIndex(i);
            }
        }
    }

    QObject::connect(selection, SIGNAL(currentIndexChanged(int)), this, SLOT(update(int)));
}

void OptionKeypointDescriptor::configChanged()
{
//    if(config.getDescriptorType() < 0) {
//        return;
//    }

    QString target = config.getDescriptorType().c_str();

    if(selection != NULL) {
        for(int i = 0; i < selection->count(); ++i) {
            if(selection->itemText(i) == target) {
                selection->setCurrentIndex(i);
                return;
            }
        }
    }
}
