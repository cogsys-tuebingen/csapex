#include "filter_local_patterns.h"

/// COMPONENT
#include <csapex/qt_helper.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <QComboBox>

PLUGINLIB_EXPORT_CLASS(vision_plugins::LocalPatterns, csapex::BoxedObject)

using namespace vision_plugins;
using namespace csapex;

LocalPatterns::LocalPatterns()
{
}

void LocalPatterns::filter(Mat &img, Mat &mask)
{

}

void LocalPatterns::insert(QBoxLayout *parent)
{
    QLabel *combo_title = new QLabel("Pattern Type:");
    parent->addWidget(combo_title);

    combo_pattern_ = new QComboBox();
    combo_pattern_->addItem("LBP");
    combo_pattern_->addItem("LTP");
    parent->addWidget(combo_pattern_);

    slider_k_ = QtHelper::makeDoubleSlider(parent, "k", 5, 0, 1000, 1.0);

    connect(slider_k_,      SIGNAL(doubleValueChanged(double)), this, SLOT(update()));
    connect(combo_pattern_, SIGNAL(currentIndexChanged(int)), this, SLOT(update()));

}

void LocalPatterns::update()
{
    state_.index = combo_pattern_->currentIndex();
    state_.k     = slider_k_->doubleValue();
}

Memento::Ptr LocalPatterns::getState() const
{
    boost::shared_ptr<State> memento(new State);
    *memento = state_;

    return memento;
}

void LocalPatterns::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    assert(m.get());

    state_ = *m;
    combo_pattern_->setCurrentIndex(state_.index);
    slider_k_->setDoubleValue(state_.k);
}

void LocalPatterns::State::readYaml(const YAML::Node &node)
{
    node["index"] >> index;
    node["k"]     >> k;
}

void LocalPatterns::State::writeYaml(YAML::Emitter &out) const
{
    out << YAML::Key << "index" << YAML::Value << index;
    out << YAML::Key << "k"     << YAML::Value << k;
}
