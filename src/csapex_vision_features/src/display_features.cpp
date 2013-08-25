/// HEADER
#include "display_features.h"

/// PROJECT
#include <utils/extractor.h>
#include <csapex/box.h>
#include <csapex/connector_out.h>
#include <csapex/connector_in.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision_features/descriptor_message.h>
#include <csapex_vision_features/keypoint_message.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::DisplayFeatures, csapex::BoxedObject)

using namespace csapex;
using namespace connection_types;

DisplayFeatures::DisplayFeatures()
    : in_key(NULL), color(cv::Scalar::all(-1)), has_img(false), has_key(false)
{
    addTag(Tag::get("Features"));
}

void DisplayFeatures::messageArrived(ConnectorIn *source)
{
    if(source == in_img) {
        has_img = true;
    } else if(source == in_key) {
        has_key = true;
    }

    if(has_key && has_img) {
        has_key = false;
        has_img = false;

        CvMatMessage::Ptr img_msg = boost::dynamic_pointer_cast<CvMatMessage> (in_img->getMessage());
        KeypointMessage::Ptr key_msg = boost::dynamic_pointer_cast<KeypointMessage> (in_key->getMessage());

        CvMatMessage::Ptr out(new CvMatMessage);
        cv::drawKeypoints(img_msg->value, key_msg->value, out->value, color, flags);

        out_img->publish(out);
    }
}

void DisplayFeatures::fill(QBoxLayout* layout)
{
    if(in_key == NULL) {

        in_img = new ConnectorIn(box_, 0);
        in_img->setType(csapex::connection_types::CvMatMessage::make());
        in_img->setLabel("Image");
        box_->addInput(in_img);

        in_key = new ConnectorIn(box_, 1);
        in_key->setType(csapex::connection_types::KeypointMessage::make());
        in_key->setLabel("Keypoints");
        box_->addInput(in_key);

        out_img = new ConnectorOut(box_, 2);
        out_img->setLabel("Image");
        box_->addOutput(out_img);

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


void DisplayFeatures::update()
{
    flags = 0;
    if(richbox->isChecked()) flags += cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS;
}

void DisplayFeatures::update(int slot)
{
    switch(slot) {
    case 0:
        color = cv::Scalar::all(-1);
        break;
    case 1:
        color = cv::Scalar::all(0);
        break;
    case 2:
        color = cv::Scalar::all(255);
        break;
    case 3:
        color = cv::Scalar(0, 0, 255);
        break;
    }
}
