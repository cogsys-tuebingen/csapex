/// HEADER
#include "filter_colorconvert.h"

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <QComboBox>
#include <QLabel>

PLUGINLIB_EXPORT_CLASS(vision_plugins::ColorConvert, csapex::BoxedObject)

using namespace vision_plugins;
using namespace csapex;

ColorConvert::ColorConvert()
{
    cs_pair_to_operation_.insert(csiPair(csPair(RGB, BGR), (int) CV_RGB2BGR));
    cs_pair_to_operation_.insert(csiPair(csPair(BGR, RGB), (int) CV_BGR2RGB));

    cs_pair_to_operation_.insert(csiPair(csPair(RGB, YUV), (int) CV_RGB2YUV));
    cs_pair_to_operation_.insert(csiPair(csPair(BGR, YUV), (int) CV_BGR2YUV));
    cs_pair_to_operation_.insert(csiPair(csPair(YUV, RGB), (int) CV_YUV2RGB));
    cs_pair_to_operation_.insert(csiPair(csPair(YUV, BGR), (int) CV_YUV2BGR));

    cs_pair_to_operation_.insert(csiPair(csPair(RGB, HSV), (int) CV_RGB2HSV));
    cs_pair_to_operation_.insert(csiPair(csPair(BGR, HSV), (int) CV_BGR2HSV));
    cs_pair_to_operation_.insert(csiPair(csPair(HSV, RGB), (int) CV_HSV2RGB));
    cs_pair_to_operation_.insert(csiPair(csPair(HSV, BGR), (int) CV_HSV2BGR));

    cs_pair_to_operation_.insert(csiPair(csPair(RGB, HSL), (int) CV_RGB2HLS));
    cs_pair_to_operation_.insert(csiPair(csPair(BGR, HSL), (int) CV_BGR2HLS));
    cs_pair_to_operation_.insert(csiPair(csPair(HSL, RGB), (int) CV_HLS2RGB));
    cs_pair_to_operation_.insert(csiPair(csPair(HSL, BGR), (int) CV_HLS2BGR));
}

ColorConvert::~ColorConvert()
{
}

void ColorConvert::insert(QBoxLayout *parent)
{
    combo_in_ = new QComboBox;
    combo_out_ = new QComboBox;
    fillCombo(combo_in_, index_to_cs_in_);
    fillCombo(combo_out_, index_to_cs_out_);

    parent->addWidget(new QLabel("Input CS"));
    parent->addWidget(combo_in_);
    parent->addWidget(new QLabel("Output CS"));
    parent->addWidget(combo_out_);
}

void ColorConvert::filter(cv::Mat &img, cv::Mat &mask)
{
    /// MEMENTO
    state_.input_index = combo_in_->currentIndex();
    state_.output_index = combo_out_->currentIndex();

    csPair cspair;
    cspair.first  = index_to_cs_in_[state_.input_index];
    cspair.second = index_to_cs_out_[state_.output_index];

    if(cspair.first == cspair.second)
        return;

    if(cs_pair_to_operation_.find(cspair) != cs_pair_to_operation_.end()) {
        int mode = cs_pair_to_operation_[cspair];
        cv::cvtColor(img, img, mode);
    } else {
        throw std::runtime_error("Conversion not supported!");
    }
}

Memento::Ptr ColorConvert::getState() const
{
    return boost::shared_ptr<State>(new State(state_));
}

void ColorConvert::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    assert(m.get());

    state_ = *m;
    combo_in_->setCurrentIndex(state_.input_index);
    combo_out_->setCurrentIndex(state_.output_index);
}

bool ColorConvert::usesMask()
{
    return false;
}

void ColorConvert::fillCombo(QComboBox *combo, std::map<int, ColorSpace> &map)
{
    combo->addItem("RGB");
    map.insert(icsPair(combo->findText("RGB"), RGB));
    combo->addItem("BGR");
    map.insert(icsPair(combo->findText("BGR"), BGR));
    combo->addItem("HSV");
    map.insert(icsPair(combo->findText("HSV"), HSV));
    combo->addItem("HSL");
    map.insert(icsPair(combo->findText("HSL"), HSL));
    combo->addItem("YUV");
    map.insert(icsPair(combo->findText("YUV"), YUV));
}
