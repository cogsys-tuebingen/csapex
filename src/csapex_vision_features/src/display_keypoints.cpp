/// HEADER
#include "display_keypoints.h"

/// PROJECT
#include <utils/extractor.h>
#include <csapex/model/box.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision_features/keypoint_message.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::DisplayKeypoints, csapex::BoxedObject)

using namespace csapex;
using namespace connection_types;

DisplayKeypoints::DisplayKeypoints()
    : in_key(NULL)
{
    state.color = cv::Scalar::all(-1);
    addTag(Tag::get("Features"));
}

void DisplayKeypoints::allConnectorsArrived()
{
    CvMatMessage::Ptr img_msg = in_img->getMessage<CvMatMessage>();
    KeypointMessage::Ptr key_msg = in_key->getMessage<KeypointMessage>();

    CvMatMessage::Ptr out(new CvMatMessage);
    cv::drawKeypoints(img_msg->value, key_msg->value, out->value, state.color, state.flags);

    out_img->publish(out);
}

void DisplayKeypoints::fill(QBoxLayout* layout)
{
    if(in_key == NULL) {
        box_->setSynchronizedInputs(true);

        in_img = box_->addInput<CvMatMessage>("Image");
        in_key = box_->addInput<KeypointMessage> ("Keypoints");

        out_img = box_->addOutput<CvMatMessage>("Image");

        colorbox = new QComboBox;
        colorbox->addItem("Random Color");
        colorbox->addItem("Black");
        colorbox->addItem("White");
        colorbox->addItem("Red");
        layout->addWidget(colorbox);
        QObject::connect(colorbox, SIGNAL(currentIndexChanged(int)), this, SLOT(update(int)));

        richbox = new QCheckBox("Rich Keypoints");
        layout->addWidget(richbox);
        QObject::connect(richbox, SIGNAL(clicked()), this, SLOT(update()));
    }
}


void DisplayKeypoints::update()
{
    state.flags = 0;
    if(richbox->isChecked()) state.flags += cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS;
}

void DisplayKeypoints::update(int slot)
{
    // TODO: cannot be deserialized in this form...
    switch(slot) {
    case 0:
        state.color = cv::Scalar::all(-1);
        break;
    case 1:
        state.color = cv::Scalar::all(0);
        break;
    case 2:
        state.color = cv::Scalar::all(255);
        break;
    case 3:
        state.color = cv::Scalar(0, 0, 255);
        break;
    }
}

Memento::Ptr DisplayKeypoints::getState() const
{
    return boost::shared_ptr<State>(new State(state));
}

void DisplayKeypoints::setState(Memento::Ptr memento)
{
    boost::shared_ptr<DisplayKeypoints::State> m = boost::dynamic_pointer_cast<DisplayKeypoints::State> (memento);
    assert(m.get());

    state = *m;

    richbox->setChecked((state.flags & cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS) != 0);
}

void DisplayKeypoints::State::writeYaml(YAML::Emitter& out) const {
    out << YAML::Key << "flags" << YAML::Value << flags;
    out << YAML::Key << "color" << YAML::Value << YAML::BeginSeq;
    out << color[0] << color[1] << color[2] << color[3];
    out << YAML::EndSeq;
}

void DisplayKeypoints::State::readYaml(const YAML::Node& node) {
    if(node.FindValue("flags")) {
        node["flags"] >> flags;
    }
    if(node.FindValue("color")) {
        const YAML::Node& seq = node["color"];
        assert(seq.Type() == YAML::NodeType::Sequence);

        int data[4];

        for(int i = 0; i < 4; ++i)  {
            seq[i] >> data[i];
        }

        color = cv::Scalar(data[0],data[1],data[2],data[3]);
    }
}
