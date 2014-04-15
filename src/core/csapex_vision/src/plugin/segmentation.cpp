/// HEADER
#include "segmentation.h"

/// PROJECT
#include <csapex/utility/qt_helper.hpp>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>


/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <boost/foreach.hpp>

CSAPEX_REGISTER_CLASS(csapex::Segmentation, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

Segmentation::Segmentation()
{
    state.min = cv::Scalar::all(0);
    state.max = cv::Scalar::all(255);

    addTag(Tag::get("Vision"));
    Tag::createIfNotExists("Segmentation");
    addTag(Tag::get("Segmentation"));
}

void Segmentation::process()
{
    CvMatMessage::Ptr img = input_img_->getMessage<CvMatMessage>();

    bool recompute = false;
    if(static_cast<unsigned>(img->getEncoding().size()) != sliders.size()) {
        recompute = true;
    } else {
        for(int i = 0, n = img->getEncoding().size(); i < n; ++i) {
            if(img->getEncoding()[i].name != state.encoding[i].name) {
                recompute = true;
                break;
            }
        }
    }

    CvMatMessage::Ptr out_mask(new CvMatMessage(enc::mono));

    if(recompute) {
        state.channels = img->value.channels();
        state.encoding = img->getEncoding();

        Q_EMIT modelChanged();

        output_mask_->publish(out_mask);
        return;
    }

    cv::Mat bw;
    cv::inRange(img->value, state.min, state.max, bw);


    if(input_mask_->isConnected()) {
        CvMatMessage::Ptr mask = input_mask_->getMessage<CvMatMessage>();
        cv::min(mask->value, bw, out_mask->value);
    } else {
        out_mask->value = bw;
    }

    output_mask_->publish(out_mask);
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

void Segmentation::fill(QBoxLayout* layout)
{
    setSynchronizedInputs(true);

    input_img_ = addInput<CvMatMessage>("Image");
    input_mask_ = addInput<CvMatMessage>("Mask", true);
    output_mask_ = addOutput<CvMatMessage>("Mask");
}

void Segmentation::updateDynamicGui(QBoxLayout *layout)
{
    sliders.clear();
    QtHelper::clearLayout(layout);
    for(int i = 0; i < state.channels; ++i) {
        QxtSpanSlider * slider = QtHelper::makeSpanSlider(layout, state.encoding[i].name,
                                                              state.min[i], state.max[i],
                                                              state.encoding[i].min, state.encoding[i].max);
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

    node["channels"] >> channels;
}
