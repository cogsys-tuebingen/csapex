/// HEADER
#include "filter_hsv_segmentation.h"

/// PROJECT
#include <vision_evaluator/qt_helper.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <boost/foreach.hpp>

PLUGINLIB_EXPORT_CLASS(cip104::HSVSegmentation, vision_evaluator::BoxedObject)

using namespace cip104;

HSVSegmentation::HSVSegmentation()
    : sliderHue(NULL)
{
    state.min = cv::Scalar::all(0);
    state.max = cv::Scalar::all(255);
}

void HSVSegmentation::filter(cv::Mat& img, cv::Mat& mask)
{
    cv::Mat hsv;
    cv::cvtColor(img, hsv, CV_BGR2HSV);

    cv::Mat bw;
    cv::inRange(hsv, state.min, state.max, bw);

    if(mask.empty()) {
        mask = bw;
    } else {
        cv::min(mask, bw, mask);
    }
}

void HSVSegmentation::update()
{
    if(!signalsBlocked()) {
        state.min = cv::Scalar(sliderHue->lowerValue(), sliderSat->lowerValue(), sliderVal->lowerValue());
        state.max = cv::Scalar(sliderHue->upperValue(), sliderSat->upperValue(), sliderVal->upperValue());
    }
}

void HSVSegmentation::insert(QBoxLayout* layout)
{
    sliderHue = HSVSegmentation::makeSpanSlider(layout, "hue", state.min[0], state.max[0], 0,255);
    sliderSat = HSVSegmentation::makeSpanSlider(layout, "sat", state.min[1], state.max[1], 0,255);
    sliderVal = HSVSegmentation::makeSpanSlider(layout, "val", state.min[2], state.max[2], 0,255);

    connect(sliderHue, SIGNAL(lowerValueChanged(int)), this, SLOT(update()));
    connect(sliderHue, SIGNAL(upperValueChanged(int)), this, SLOT(update()));
    connect(sliderSat, SIGNAL(lowerValueChanged(int)), this, SLOT(update()));
    connect(sliderSat, SIGNAL(upperValueChanged(int)), this, SLOT(update()));
    connect(sliderVal, SIGNAL(lowerValueChanged(int)), this, SLOT(update()));
    connect(sliderVal, SIGNAL(upperValueChanged(int)), this, SLOT(update()));
}


Memento::Ptr HSVSegmentation::getState() const
{
    boost::shared_ptr<State> memento(new State);
    *memento = state;

    return memento;
}

void HSVSegmentation::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    assert(m.get());

    state = *m;

    blockSignals(true);
    sliderHue->setLowerValue(state.min[0]);
    sliderHue->setUpperValue(state.max[0]);
    sliderSat->setLowerValue(state.min[1]);
    sliderSat->setUpperValue(state.max[1]);
    sliderVal->setLowerValue(state.min[2]);
    sliderVal->setUpperValue(state.max[2]);
    blockSignals(false);

    Q_EMIT filter_changed();
}

void HSVSegmentation::State::writeYaml(YAML::Emitter& out) const {
    out << YAML::Key << "min" << YAML::Value;
    out << YAML::BeginSeq << min[0] << min[1] << min[2] << YAML::EndSeq;
    out << YAML::Key << "max" << YAML::Value;
    out << YAML::BeginSeq << max[0] << max[1] << max[2] << YAML::EndSeq;
}

void HSVSegmentation::State::readYaml(const YAML::Node& node) {
    const YAML::Node& min_ = node["min"];
    const YAML::Node& max_ = node["max"];
    assert(min_.Type() == YAML::NodeType::Sequence);
    assert(max_.Type() == YAML::NodeType::Sequence);

    for(int i = 0; i < 3; ++i) {
        min_[i] >> min[i];
        max_[i] >> max[i];
    }
    std::cout << max[0]<<std::endl;
}


QxtSpanSlider* HSVSegmentation::makeSpanSlider(QBoxLayout* layout, const std::string& name, int lower, int upper, int min, int max) {
    QHBoxLayout* internal_layout = new QHBoxLayout;

    QxtSpanSlider* slider = new QxtSpanSlider(Qt::Horizontal);
    slider->setMinimum(min);
    slider->setMaximum(max);
    slider->setLowerValue(lower);
    slider->setUpperValue(upper);
    slider->setMinimumWidth(128);

    QWrapper::QSpinBoxExt* displayLower = new QWrapper::QSpinBoxExt;
    displayLower->setMinimum(min);
    displayLower->setMaximum(max);
    displayLower->setValue(lower);

    QWrapper::QSpinBoxExt* displayUpper = new QWrapper::QSpinBoxExt;
    displayUpper->setMinimum(min);
    displayUpper->setMaximum(max);
    displayUpper->setValue(upper);

    internal_layout->addWidget(new QLabel(name.c_str()));
    internal_layout->addWidget(displayLower);
    internal_layout->addWidget(slider);
    internal_layout->addWidget(displayUpper);

    layout->addLayout(internal_layout);

    QObject::connect(slider,        SIGNAL(rangeChanged(int,int)),  displayUpper,   SLOT(setRange(int,int)));
    QObject::connect(slider,        SIGNAL(lowerValueChanged(int)), displayLower,   SLOT(setValue(int)));
    QObject::connect(slider,        SIGNAL(upperValueChanged(int)), displayUpper,   SLOT(setValue(int)));
    QObject::connect(displayLower,  SIGNAL(valueChanged(int)),      slider,         SLOT(setLowerValue(int)));
    QObject::connect(displayUpper,  SIGNAL(valueChanged(int)),      slider,         SLOT(setUpperValue(int)));

    return slider;
}
