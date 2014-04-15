/// HEADER
#include "bilateral.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_vision/cv_mat_message.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>


CSAPEX_REGISTER_CLASS(csapex::BilateralFilter, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

BilateralFilter::BilateralFilter() :
    d_(1),
    sigma_color_(0.0),
    sigma_space_(0.0)
{
    addTag(Tag::get("Filter"));
    addTag(Tag::get("Vision"));
    addParameter(param::ParameterFactory::declareRange("d", 1, 255, d_, 1),
                 boost::bind(&BilateralFilter::update, this));
    addParameter(param::ParameterFactory::declareRange("sigma color", -255.0, 255.0, sigma_color_, 0.1),
                 boost::bind(&BilateralFilter::update, this));
    addParameter(param::ParameterFactory::declareRange("sigma space", -255.0, 255.0, sigma_space_, 0.1),
                 boost::bind(&BilateralFilter::update, this));
}

void BilateralFilter::process()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(enc::bgr));


    cv::Mat tmp = in->value.clone();
    std::vector<cv::Mat> channels;
    switch(tmp.type()) {
    case CV_8UC1:
    case CV_8UC3:
        break;
    case CV_8UC4:
        cv::split(tmp, channels);
        channels.pop_back();
        cv::merge(channels,tmp);
        tmp.convertTo(tmp, CV_8UC3);
        break;
    default:
        throw std::runtime_error("CV_8UC1 or CV_8UC3 required!");
    }

    out->value  = cv::Mat(tmp.rows, tmp.cols, tmp.type(), cv::Scalar::all(0));
    cv::bilateralFilter(tmp, out->value, d_, sigma_color_, sigma_space_);
    output_->publish(out);
}

void BilateralFilter::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<CvMatMessage>("Unblurred");
    output_ = addOutput<CvMatMessage>("Blurred");

    update();
}

void BilateralFilter::update()
{
    d_           = param<int>("d");
    sigma_color_ = param<double>("sigma color");
    sigma_space_ = param<double>("sigma space");
}
