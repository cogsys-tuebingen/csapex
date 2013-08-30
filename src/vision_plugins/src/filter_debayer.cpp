/// HEADER
#include "filter_debayer.h"

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <QComboBox>
#include <QLabel>

PLUGINLIB_EXPORT_CLASS(vision_plugins::Debayer, csapex::BoxedObject)

using namespace vision_plugins;
using namespace csapex;

Debayer::Debayer()
    : csapex::Filter()
{
    setIcon(QIcon(":/bayer.png"));
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

    // assume 1 channel raw image comes in
    cv::Mat raw;
    cv::cvtColor(img, raw, CV_RGB2GRAY);

    if (mode == 667) {
        this->debayerAndResize(raw, img);
        cv::cvtColor(img, img, CV_BGR2RGB);
    }
    else {
        cv::cvtColor(raw, img, mode);
    }
}

Memento::Ptr Debayer::getState() const
{
    return boost::shared_ptr<State>(new State(state_));
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
    combo->addItem("NNRG2RGB");
    map.insert(modePair(combo->findText("NNRG2RGB"), 667));
}

// Debayer: bayer-Pattern
// - every pixel has it's own color filter (e.g. only sees red)
// - pixel returns brightness value
void Debayer::debayerAndResize(cv::Mat& source, cv::Mat& dest) {

    cv::MatIterator_<uchar> it = source.begin<uchar>(),
                         itEnd = source.end<uchar>();
    uchar* destination = (uchar*) dest.data;

    while(it != itEnd) {
        // r g r g r g
        // g b g b g b
        cv::MatIterator_<uchar> itLineEnd = it + 640;
        while(it != itLineEnd) {
            *destination = *it;
            ++it;
            ++destination;
            *destination = *it;
            ++it;
            ++destination;
            ++destination;
        }
        itLineEnd = it + 640;
        destination -= 320*3;
        while(it != itLineEnd) {
            // maybe add some green
            ++it;
            ++destination;
            ++destination;
            // add blue
            *destination = *it;
            ++it;
            ++destination;
        }
        destination += 320*3;
    }
}

void Debayer::State::readYaml(const YAML::Node &node)
{
    node["input_index"] >> index;
}

void Debayer::State::writeYaml(YAML::Emitter &out) const
{
    out << YAML::Key << "input_index" << YAML::Value << index;
}
