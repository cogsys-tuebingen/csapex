/// HEADER
#include "filter_coloradjustment.h"

/// COMPONENT
#include <csapex/qt_helper.hpp>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/connector_in.h>
#include <csapex/connector_out.h>
#include <csapex/box.h>
#include <utils/LibCvTools/histogram.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <QComboBox>
#include <QCheckBox>

PLUGINLIB_EXPORT_CLASS(csapex::ColorAdjustment, csapex::BoxedObject)

using namespace csapex;
using namespace connection_types;

ColorAdjustment::ColorAdjustment() :
    input_(NULL),
    output_(NULL),
    active_preset_(NONE),
    container_ch_sliders_(NULL)
{
    addTag(Tag::get("Filter"));
}

ColorAdjustment::~ColorAdjustment()
{
}

void ColorAdjustment::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    assert(m.get());

    state_ = *m;
    combo_preset_->setCurrentIndex(state_.combo_index);
    setPreset(state_.combo_index);

    Q_EMIT modelChanged();
}

Memento::Ptr ColorAdjustment::getState() const
{
    boost::shared_ptr<State> memento(new State);
    *memento = state_;

    return memento;
}

void ColorAdjustment::fill(QBoxLayout *parent)
{
    if(input_ == NULL || output_ == NULL) {
        /// add input
        input_ = new ConnectorIn(box_, 0);
        box_->addInput(input_);

        /// add output
        output_ = new ConnectorOut(box_, 0);
        box_->addOutput(output_);

        check_normalize_ = new QCheckBox("Normalization Mode");
        parent->addWidget(check_normalize_);


        parent->addWidget(new QLabel("Channel Adjustment"));

        combo_preset_ = new QComboBox();
        combo_preset_->addItem("STD");
        combo_preset_->addItem("HSV");
        combo_preset_->addItem("HSL");

        int index = combo_preset_->findText("STD");
        index_to_preset_.insert(intPresetPair(index, STD));
        index = combo_preset_->findText("HSV");
        index_to_preset_.insert(intPresetPair(index, HSV));
        index = combo_preset_->findText("HSL");
        index_to_preset_.insert(intPresetPair(index, HSL));
        parent->addWidget(combo_preset_);

        setPreset(combo_preset_->currentIndex());
        slide_lightness_ = QtHelper::makeSlider(parent, "Lightness -/+", 0, -255, 255);

        QObject::connect(combo_preset_, SIGNAL(currentIndexChanged(int)), this, SLOT(setPreset(int)));
    }
}


void ColorAdjustment::messageArrived(ConnectorIn *source)
{
    CvMatMessage::Ptr m = boost::shared_dynamic_cast<CvMatMessage>(source->getMessage());

    std::vector<cv::Mat> channels;
    cv::split(m->value, channels);

    if(state_.channel_count != m->value.channels()) {
        state_.channel_count = m->value.channels();
        Q_EMIT modelChanged();
    }

    updateState();

    for(int i = 0 ; i < slider_pairs_.size() && i < channels.size() ; i++) {
        double min = slider_pairs_[i].first->doubleValue();
        double max = slider_pairs_[i].second->doubleValue();
        if(check_normalize_->isChecked()) {
            cv::normalize(channels[i], channels[i], max, min, cv::NORM_MINMAX);
        } else {
            cv::threshold(channels[i], channels[i], min, max, cv::THRESH_TOZERO);
            cv::threshold(channels[i], channels[i], max, max, cv::THRESH_TOZERO_INV);
        }

    }

    cv::merge(channels, m->value);
    addLightness(m->value);

    output_->publish(m);
}

void ColorAdjustment::updateDynamicGui(QBoxLayout *layout)
{
    slider_pairs_.clear();
    QVBoxLayout *internal_layout;

    if(container_ch_sliders_ != NULL) {
        container_ch_sliders_->deleteLater();
    }

    internal_layout = new QVBoxLayout;

    for(int i = 0 ; i < state_.channel_count ; i++) {
        std::stringstream ch;
        ch << i + 1;

        double ch_limit = 255.0;
        if(i == 0 && (active_preset_ == HSL || active_preset_ == HSV)) {
            ch_limit = 180.0;
        }

        if(state_.mins.size() < state_.channel_count) {
                state_.mins.push_back(0.0);
        }

        if(state_.maxs.size() < state_.channel_count) {
            state_.maxs.push_back(ch_limit);
        }

        QDoubleSlider *tmp_min = QtHelper::makeDoubleSlider(internal_layout, "Ch." + ch.str() + " min.", state_.mins[i], 0.0, ch_limit, 0.01);
        QDoubleSlider *tmp_max = QtHelper::makeDoubleSlider(internal_layout, "Ch." + ch.str() + " max.", state_.maxs[i], 0.0, ch_limit, 0.01);
        prepareSliderPair(tmp_min, tmp_max);

    }

    slide_lightness_->setValue(state_.lightness);
    check_normalize_->setChecked(state_.normalize);

    container_ch_sliders_ = QtHelper::wrapLayout(internal_layout);
    layout->addWidget(container_ch_sliders_);

}

void ColorAdjustment::setPreset(int index)
{
    active_preset_  = index_to_preset_[index];
    Q_EMIT modelChanged();
}

void ColorAdjustment::addLightness(cv::Mat &img)
{
    double      slide_value = slide_lightness_->value();
    slide_value = std::abs(slide_value);

    cv::Scalar  cv_value;
    cv_value[0] = slide_value;
    cv_value[1] = active_preset_ == HSL || active_preset_ == HSV ? 0 : slide_value;
    cv_value[2] = active_preset_ == HSL || active_preset_ == HSV ? 0 : slide_value;
    cv_value[3] = slide_value;

    cv::Mat     lightness(img.rows, img.cols, img.type(), cv_value);
    if(slide_lightness_->value() < 0) {
        cv::subtract(img, lightness, img);
    } else if(slide_lightness_->value() > 0) {
        cv::add(img, lightness, img);
    }
}

void ColorAdjustment::prepareSliderPair(QDoubleSlider *min, QDoubleSlider *max)
{
    std::pair<QDoubleSlider*, QDoubleSlider*> minMax;
    minMax.first  = min;
    minMax.second = max;
    QDoubleSlider::connect(minMax.first, SIGNAL(valueChanged(double)), minMax.second, SLOT(limitMin(double)));
    QDoubleSlider::connect(minMax.second, SIGNAL(valueChanged(double)), minMax.first, SLOT(limitMax(double)));
    slider_pairs_.push_back(minMax);
}

void ColorAdjustment::updateState()
{
    state_.mins.clear();
    state_.maxs.clear();
    state_.lightness = slide_lightness_->value();
    state_.combo_index = combo_preset_->currentIndex();
    state_.normalize = check_normalize_->isChecked();

    for(ChannelLimits::iterator it = slider_pairs_.begin() ; it != slider_pairs_.end() ; it++) {
        state_.mins.push_back(it->first->doubleValue());
        state_.maxs.push_back(it->second->doubleValue());
    }
}

/// MEMENTO ------------------------------------------------------------------------------------
ColorAdjustment::State::State() :
    channel_count(0),
    lightness(0)
{
}

void ColorAdjustment::State::readYaml(const YAML::Node &node)
{
    const YAML::Node &min_values = node["mins"];
    for(YAML::Iterator it = min_values.begin() ; it != min_values.end() ; it++) {
        double min;
        *it >> min;
        mins.push_back(min);
    }

    const YAML::Node &max_values = node["maxs"];
    for(YAML::Iterator it = max_values.begin() ; it != max_values.end() ; it++) {
        double max;
        *it >> max;
        maxs.push_back(max);
    }

    node["channel_count"] >> channel_count;
    node["normalize"] >> normalize;
    node["lightness"] >> lightness;
    node["preset"] >> combo_index;
}


void ColorAdjustment::State::writeYaml(YAML::Emitter &out) const
{
    out << YAML::Key << "mins" << YAML::Value << YAML::BeginSeq;
    for(std::vector<double>::const_iterator it = mins.begin() ; it != mins.end() ; it++) {
        out << *it;
    }
    out << YAML::EndSeq;
    out << YAML::Key << "maxs" << YAML::Value << YAML::BeginSeq;
    for(std::vector<double>::const_iterator it = maxs.begin() ; it != maxs.end() ; it++) {
        out << *it;
    }
    out << YAML::EndSeq;
    out << YAML::Key << "channel_count" << YAML::Value << channel_count;
    out << YAML::Key << "normalize" << YAML::Value << normalize;
    out << YAML::Key << "preset" << YAML::Value << combo_index;
    out << YAML::Key << "lightness" << YAML::Value << lightness;
}
