/// HEADER
#include "display_keypoints.h"

/// PROJECT
#include <utils/extractor.h>

#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision_features/keypoint_message.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::DisplayKeypoints, csapex::Node)

using namespace csapex;
using namespace connection_types;

DisplayKeypoints::DisplayKeypoints()
    : in_key(NULL)
{
    addTag(Tag::get("Features"));

    addParameter(param::ParameterFactory::declare("rich keypoints", true));
    addParameter(param::ParameterFactory::declareColorParameter("color", 255,0,0));
    addParameter(param::ParameterFactory::declare("random color", true));
}

void DisplayKeypoints::allConnectorsArrived()
{
    CvMatMessage::Ptr img_msg = in_img->getMessage<CvMatMessage>();
    KeypointMessage::Ptr key_msg = in_key->getMessage<KeypointMessage>();

    CvMatMessage::Ptr out(new CvMatMessage);


    cv::Scalar color(-1,-1,-1,-1);
    if(!param<bool>("random color")) {
        const std::vector<int>& c = param<std::vector<int> >("color");
        color = cv::Scalar(c[2], c[1], c[0]);
    }

    int flags = 0;
    if(param<bool>("rich keypoints")) {
        flags |= cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS;
    }

    cv::drawKeypoints(img_msg->value, key_msg->value, out->value, color, flags);

    out_img->publish(out);
}

void DisplayKeypoints::setup()
{
    setSynchronizedInputs(true);

    in_img = addInput<CvMatMessage>("Image");
    in_key = addInput<KeypointMessage> ("Keypoints");

    out_img = addOutput<CvMatMessage>("Image");
}
