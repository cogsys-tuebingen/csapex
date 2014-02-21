/// HEADER
#include "blur.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_vision/cv_mat_message.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>


CSAPEX_REGISTER_CLASS(csapex::BoxBlur, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

BoxBlur::BoxBlur() :
    kernel_(1)
{
    addTag(Tag::get("Filter"));
    addTag(Tag::get("Vision"));
    addParameter(param::ParameterFactory::declareRange("kernel", 1, 255, kernel_, 2),
                 boost::bind(&BoxBlur::update, this));
}

void BoxBlur::process()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding()));

    cv::blur(in->value,out->value, cv::Size(kernel_, kernel_));

    output_->publish(out);
}

void BoxBlur::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<CvMatMessage>("Unblurred");
    output_ = addOutput<CvMatMessage>("Blurred");

    update();
}

void BoxBlur::update()
{
    kernel_ = param<int>("kernel");
}
