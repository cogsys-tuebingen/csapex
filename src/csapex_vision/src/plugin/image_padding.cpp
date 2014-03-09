/// HEADER
#include "image_padding.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_vision/cv_mat_message.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::ImagePadding, csapex::Node)


using namespace csapex;
using namespace connection_types;

ImagePadding::ImagePadding()
{
    addTag(Tag::get("General"));
    addTag(Tag::get("Vision"));

    setIcon(QIcon(":/border.png"));

    addParameter(param::ParameterFactory::declareRange("border", 0, 1000, 0, 1));
    addParameter(param::ParameterFactory::declareRange("mask offset", 0, 100, 0, 1));
}

void ImagePadding::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<CvMatMessage>("Image");

    output_ = addOutput<CvMatMessage>("Expanded Image");
    output_mask_ = addOutput<CvMatMessage>("Expanded Mask");
}

void ImagePadding::process()
{
    if(!output_->isConnected() && !output_mask_->isConnected()) {
        return;
    }

    CvMatMessage::Ptr img_msg = input_->getMessage<CvMatMessage>();

    int rows = img_msg->value.rows;
    int cols = img_msg->value.cols;

    int border = param<int>("border");

    if(output_->isConnected()) {
        CvMatMessage::Ptr result(new CvMatMessage(img_msg->getEncoding()));
        result->value = cv::Mat(rows + 2 * border, cols + 2 * border, img_msg->value.type(), cv::Scalar::all(0));
        cv::Mat roi(result->value, cv::Rect(border, border, cols, rows));

        img_msg->value.copyTo(roi);

        output_->publish(result);
    }

    if(output_mask_->isConnected()) {
        CvMatMessage::Ptr result(new CvMatMessage(enc::mono));

        result->value = cv::Mat(rows + 2 * border, cols + 2 * border, CV_8UC1, cv::Scalar::all(0));

        int mask_offset = param<int>("mask offset");
        cv::Rect roi_rect(border+mask_offset, border+mask_offset, cols-2*mask_offset, rows-2*mask_offset);
        cv::rectangle(result->value, roi_rect, cv::Scalar::all(255), CV_FILLED);

        output_mask_->publish(result);
    }

}
