/// HEADER
#include "segmentation.h"

/// PROJECT
#include <csapex/utility/qt_helper.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <boost/foreach.hpp>

PLUGINLIB_EXPORT_CLASS(csapex::Segmentation, csapex::BoxedObject)

using namespace csapex;

Segmentation::Segmentation()
{
    state.min = cv::Scalar::all(0);
    state.max = cv::Scalar::all(255);
}

void Segmentation::filter(cv::Mat& img, cv::Mat& mask)
{
    if(static_cast<unsigned>(img.channels()) != sliders.size()) {
        state.channels = img.channels();
        Q_EMIT modelChanged();
        return;
    }

    cv::Mat bw;
    cv::inRange(img, state.min, state.max, bw);

    if(mask.empty()) {
        mask = bw;
    } else {
        cv::min(mask, bw, mask);
    }
}

void Segmentation::update()
{
    if(!signalsBlocked()) {
        state.min = cv::Scalar::all(0);
        state.max = cv::Scalar::all(0);

        for(int i = 0, n = sliders.size(); i < n; ++i) {
            state.min[i] = sliders[i]->lowerValue();
            state.max[i] = sliders[i]->upperValue();
        }
    }
}

void Segmentation::insert(QBoxLayout* layout)
{
}

void Segmentation::updateDynamicGui(QBoxLayout *layout)
{
    sliders.clear();
    for(int i = 0; i < state.channels; ++i) {
        std::stringstream name;
        name << "channel " << i;
        QxtSpanSlider * slider = Segmentation::makeSpanSlider(layout, name.str().c_str(), state.min[i], state.max[i], 0,255);
        connect(slider, SIGNAL(lowerValueChanged(int)), this, SLOT(update()));
        connect(slider, SIGNAL(upperValueChanged(int)), this, SLOT(update()));

        sliders.push_back(slider);
    }
}


Memento::Ptr Segmentation::getState() const
{
    return boost::shared_ptr<State>(new State(state));
}

void Segmentation::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    assert(m.get());

    state = *m;

    Q_EMIT filter_changed();
}

void Segmentation::State::writeYaml(YAML::Emitter& out) const {
    out << YAML::Key << "channels" << YAML::Value << channels;
    out << YAML::Key << "min" << YAML::Value;
    out << YAML::BeginSeq << min[0] << min[1] << min[2] << YAML::EndSeq;
    out << YAML::Key << "max" << YAML::Value;
    out << YAML::BeginSeq << max[0] << max[1] << max[2] << YAML::EndSeq;
}

void Segmentation::State::readYaml(const YAML::Node& node) {
    const YAML::Node& min_ = node["min"];
    const YAML::Node& max_ = node["max"];
    assert(min_.Type() == YAML::NodeType::Sequence);
    assert(max_.Type() == YAML::NodeType::Sequence);

    for(int i = 0; i < 3; ++i) {
        min_[i] >> min[i];
        max_[i] >> max[i];
    }
    std::cout << max[0]<<std::endl;

    node["channels"] >> channels;
}


QxtSpanSlider* Segmentation::makeSpanSlider(QBoxLayout* layout, const std::string& name, int lower, int upper, int min, int max) {
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
