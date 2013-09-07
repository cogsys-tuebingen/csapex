/// HEADER
#include "image_padding.h"

/// PROJECT
#include <csapex/qt_helper.hpp>
#include <csapex/connector_in.h>
#include <csapex/connector_out.h>
#include <csapex/box.h>
#include <csapex_vision/cv_mat_message.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::ImagePadding, csapex::BoxedObject)


using namespace csapex;
using namespace connection_types;

ImagePadding::ImagePadding()
    : slider(NULL)
{
    state.border = 0;

    addTag(Tag::get("General"));
    addTag(Tag::get("Vision"));

    setIcon(QIcon(":/border.png"));
}

void ImagePadding::fill(QBoxLayout *layout)
{
    if(slider == NULL) {
        slider = QtHelper::makeSlider(layout, "border width", 0, 0, 500);
        connect(slider, SIGNAL(valueChanged(int)), this, SLOT(update()));

        input_ = new ConnectorIn(box_, 0);
        input_->setLabel("Image");
        box_->addInput(input_);

        output_ = new ConnectorOut(box_, 0);
        output_->setLabel("Expanded Image");
        box_->addOutput(output_);

        output_mask_ = new ConnectorOut(box_, 1);
        output_mask_->setLabel("Expanded Mask");
        box_->addOutput(output_mask_);
    }
}

void ImagePadding::update()
{
    state.border = slider->value();
}

void ImagePadding::messageArrived(ConnectorIn *source)
{
    if(!output_->isConnected() && !output_mask_->isConnected()) {
        return;
    }

    ConnectionType::Ptr msg = input_->getMessage();
    CvMatMessage::Ptr img_msg = boost::dynamic_pointer_cast<CvMatMessage> (msg);


    int rows = img_msg->value.rows;
    int cols = img_msg->value.cols;

    if(output_->isConnected()) {
        CvMatMessage::Ptr result(new CvMatMessage);
        result->value = cv::Mat(rows + 2 * state.border, cols + 2 * state.border, img_msg->value.type(), cv::Scalar::all(0));
        cv::Mat roi(result->value, cv::Rect(state.border, state.border, cols, rows));

        img_msg->value.copyTo(roi);

        output_->publish(result);
    }

    if(output_mask_->isConnected()) {
        CvMatMessage::Ptr result(new CvMatMessage);

        result->value = cv::Mat(rows + 2 * state.border, cols + 2 * state.border, CV_8UC1, cv::Scalar::all(0));

        // TODO: make this a parameter
        int mask_offset = 0;
        cv::Rect roi_rect(state.border+mask_offset, state.border+mask_offset, cols-2*mask_offset, rows-2*mask_offset);
        cv::rectangle(result->value, roi_rect, cv::Scalar::all(255), CV_FILLED);

        output_mask_->publish(result);
    }

}

Memento::Ptr ImagePadding::getState() const
{
    return boost::shared_ptr<State>(new State(state));
}

void ImagePadding::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    assert(m.get());

    state = *m;

    slider->setValue(state.border);
}


void ImagePadding::State::writeYaml(YAML::Emitter& out) const
{
    out << YAML::Key << "border" << YAML::Value << border;
}

void ImagePadding::State::readYaml(const YAML::Node& node)
{
    if(node.FindValue("border")) {
        node["border"] >> border;
    }
}