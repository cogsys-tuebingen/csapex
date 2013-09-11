/// HEADER
#include "option_keypoint_extractor.h"

/// PROJECT
#include <utils/extractor_manager.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <csapex/utility/qt_helper.hpp>
#include <boost/foreach.hpp>

PLUGINLIB_EXPORT_CLASS(robot_detection::OptionKeypointExtractor, csapex::GlobalOption)

using namespace robot_detection;
using namespace csapex;

OptionKeypointExtractor::OptionKeypointExtractor()
    : selection(NULL), threshold(NULL)
{
}

OptionKeypointExtractor::~OptionKeypointExtractor()
{
}

void OptionKeypointExtractor::update_type(int slot)
{
    Config config = Config::instance();

    QString target = config("keypointType").as<std::string>().c_str();
    if(target == selection->itemText(slot)) {
        return;
    }

    config.setKeypointType(selection->itemText(slot).toStdString());
    config.replaceInstance();
}

void OptionKeypointExtractor::update_threshold(int t)
{
    Config current = Config::instance();
    current["extractor_threshold"] = t;
    current.replaceInstance();
}

void OptionKeypointExtractor::insert(QBoxLayout* layout)
{
    selection = new QComboBox;

    ExtractorManager& manager = ExtractorManager::instance();
    typedef std::pair<std::string, ExtractorManager::ExtractorInitializer> Pair;
    BOOST_FOREACH(Pair fc, manager.featureDetectors()) {
        selection->addItem(fc.second.getType().c_str());
    }
    layout->addLayout(QtHelper::wrap("Keypoint", selection));

    Config config = Config::instance();
    QString target = config("keypointType").as<std::string>().c_str();

    if(selection != NULL) {
        for(int i = 0; i < selection->count(); ++i) {
            if(selection->itemText(i) == target) {
                selection->setCurrentIndex(i);
            }
        }
    }


    QObject::connect(selection, SIGNAL(currentIndexChanged(int)), this, SLOT(update_type(int)));

    threshold = QtHelper::makeSlider(layout, "threshold", config("extractor_threshold"), 1, 200);
    layout->addWidget(threshold);

    QObject::connect(threshold, SIGNAL(valueChanged(int)), this, SLOT(update_threshold(int)));
}

void OptionKeypointExtractor::configChanged()
{
//    if(config("keypointType") < 0) {
//        return;
//    }

    QString target = config("keypointType").as<std::string>().c_str();

    threshold->setValue(config("extractor_threshold"));
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
struct OptionKeypointExtractorState : public Memento {
    typedef boost::shared_ptr<OptionKeypointExtractorState> Ptr;
    std::string keypoint_name;
    int threshold;

    virtual void writeYaml(YAML::Emitter& out) const {
        out << YAML::Key << "keypoint_name" << YAML::Value << keypoint_name;
        out << YAML::Key << "threshold" << YAML::Value << threshold;
    }
    virtual void readYaml(const YAML::Node& node) {
        node["threshold"] >> threshold;
        node["keypoint_name"] >> keypoint_name;
    }
};
}

Memento::Ptr OptionKeypointExtractor::getState() const
{
    Config current = Config::instance();

    OptionKeypointExtractorState::Ptr res(new OptionKeypointExtractorState);
    res->threshold = current("extractor_threshold");
    res->keypoint_name = current("keypointType").as<std::string>();

    return res;
}

void OptionKeypointExtractor::setState(Memento::Ptr memento)
{
    OptionKeypointExtractorState::Ptr m = boost::dynamic_pointer_cast<OptionKeypointExtractorState> (memento);
    assert(m.get());

    Config current = Config::instance();
    current["extractor_threshold"] = m->threshold;
    current["keypointType"] = m->keypoint_name;
    current.replaceInstance();

    threshold->setValue(m->threshold);
}
