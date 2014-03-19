/// HEADER
#include "output_display.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <utils_qt/QtCvImageConverter.h>
#include <csapex_vision/cv_mat_message.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::OutputDisplay, csapex::Node)


using namespace csapex;
using namespace connection_types;

OutputDisplay::OutputDisplay()
{
    addTag(Tag::get("General"));
    addTag(Tag::get("Vision"));

    setIcon(QIcon(":/picture.png"));
}

OutputDisplay::~OutputDisplay()
{
}

void OutputDisplay::setup()
{
    setSynchronizedInputs(true);
    input_ = addInput<CvMatMessage>("Image", false, true);
}

void OutputDisplay::process()
{
    CvMatMessage::Ptr mat_msg = input_->getMessage<CvMatMessage>();

    if(mat_msg.get() && !mat_msg->value.empty()) {
        QSharedPointer<QImage> img = QtCvImageConverter::Converter<QImage, QSharedPointer>::mat2QImage(mat_msg->value);
        display_request(img);
    }
}
