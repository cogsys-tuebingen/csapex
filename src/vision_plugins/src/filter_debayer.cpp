/// HEADER
#include "filter_debayer.h"

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <QComboBox>
#include <QLabel>

PLUGINLIB_EXPORT_CLASS(vision_plugins::Debayer, vision_evaluator::BoxedObject)

using namespace vision_plugins;

Debayer::Debayer()
{
}

Debayer::~Debayer()
{
}

void Debayer::insert(QBoxLayout *parent)
{
    combo_mode_ = new QComboBox;
    fillCombo(combo_mode_, modeFromCombo);

    parent->addWidget(new QLabel("Input CS"));
    parent->addWidget(combo_mode_);
}

void Debayer::filter(cv::Mat &img, cv::Mat &mask)
{
    /// MEMENTO
    state_.index = combo_mode_->currentIndex();

    int mode = modeFromCombo[state_.index];

    cv::Mat gray;
    cv::cvtColor(img, gray, CV_RGB2GRAY);
    cv::cvtColor(gray, img, mode);
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

void Debayer::fillCombo(QComboBox *combo, std::map<int, int> &map)
{
    combo->addItem("BayerBG2RGB");
    map.insert(modePair(combo->findText("BayerBG2RGB"), CV_BayerBG2RGB));
    combo->addItem("BayerGB2RGB");
    map.insert(modePair(combo->findText("BayerGB2RGB"), CV_BayerGB2RGB));
    combo->addItem("BayerRG2RGB");
    map.insert(modePair(combo->findText("BayerRG2RGB"), CV_BayerRG2RGB));
    combo->addItem("BayerGR2RGB");
    map.insert(modePair(combo->findText("BayerGR2RGB"), CV_BayerGR2RGB));
}
