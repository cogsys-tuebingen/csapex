/// HEADER
#include "filter_debayer.h"

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <QComboBox>
#include <QLabel>

PLUGINLIB_EXPORT_CLASS(vision_plugins::Debayer, vision_evaluator::BoxedObject)

using namespace vision_plugins;

void Debayer::insert(QBoxLayout *parent)
{
    combo_mode_ = new QComboBox;
    fillCombo(combo_mode_);

    parent->addWidget(new QLabel("Input CS"));
    parent->addWidget(combo_mode_);
}

void Debayer::filter(cv::Mat &img, cv::Mat &mask)
{
    /// MEMENTO
    state_.index = combo_mode_->currentIndex();

    cv::Mat gray;
    cv::cvtColor(img, gray, CV_RGB2GRAY);
    cv::cvtColor(gray, img, state_.index);
}

Memento::Ptr Debayer::getState() const
{
    boost::shared_ptr<State> memento(new State);
    *memento = state_;

    return memento;
}

void Debayer::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    assert(m.get());

    state_ = *m;
    combo_mode_->setCurrentIndex(state_.index);
}

bool Debayer::usesMask()
{
    return false;
}

void Debayer::fillCombo(QComboBox *combo)
{
    combo->addItem("BayerBG2RGB");
    combo->addItem("BayerGB2RGB");
    combo->addItem("BayerRG2RGB");
    combo->addItem("BayerGR2RGB");
}

void Debayer::State::readYaml(const YAML::Node &node)
{
    node["input_index"] >> index;
}

void Debayer::State::writeYaml(YAML::Emitter &out) const
{
    out << YAML::Key << "input_index" << YAML::Value << index;
}
