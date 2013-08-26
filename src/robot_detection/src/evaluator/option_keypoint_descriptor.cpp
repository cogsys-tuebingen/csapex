/// HEADER
#include "option_keypoint_descriptor.h"

/// PROJECT
#include <utils/extractor_manager.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <csapex/qt_helper.hpp>

PLUGINLIB_EXPORT_CLASS(robot_detection::OptionKeypointDescriptor, csapex::GlobalOption)

using namespace robot_detection;
using namespace csapex;


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

    config.setDescriptorType(selection->itemText(slot).toStdString());
    config.replaceGlobal();
}

void OptionKeypointDescriptor::insert(QBoxLayout* layout)
{
    selection = new QComboBox;

    ExtractorManager& manager = ExtractorManager::instance();
    typedef std::pair<std::string, ExtractorManager::ExtractorInitializer> Pair;
    BOOST_FOREACH(Pair fc, manager.descriptorExtractors()) {
        selection->addItem(fc.second.getType().c_str());
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

namespace {
struct OptionKeypointDescriptorState : public Memento {
    typedef boost::shared_ptr<OptionKeypointDescriptorState> Ptr;
    std::string descriptor_name;

    virtual void writeYaml(YAML::Emitter& out) const {
        out << YAML::Key << "descriptor_name" << YAML::Value << descriptor_name;
    }
    virtual void readYaml(const YAML::Node& node) {
        node["descriptor_name"] >> descriptor_name;
    }
};
}

Memento::Ptr OptionKeypointDescriptor::getState() const
{
    Config current = Config::getGlobal();

    OptionKeypointDescriptorState::Ptr res(new OptionKeypointDescriptorState);
    res->descriptor_name = current.descriptor_name;
    return res;
}

void OptionKeypointDescriptor::setState(Memento::Ptr memento)
{
    OptionKeypointDescriptorState::Ptr m = boost::dynamic_pointer_cast<OptionKeypointDescriptorState> (memento);
    assert(m.get());

    Config current = Config::getGlobal();
    current.descriptor_name = m->descriptor_name;
    current.replaceGlobal();
}

