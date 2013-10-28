/// HEADER
#include "camera.h"

/// COMPONENT
#include <csapex_vision/cv_mat_message.h>

/// PROJECT
#include <csapex/utility/qt_helper.hpp>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>


/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>


CSAPEX_REGISTER_CLASS(csapex::Camera, csapex::BoxedObject)

using namespace csapex;

Camera::Camera()
{
}

void Camera::tick()
{
    if(cap_.isOpened()) {
        connection_types::CvMatMessage::Ptr msg(new connection_types::CvMatMessage);
        cap_ >> msg->value;
        output_->publish(msg);
    }
}

void Camera::fill(QBoxLayout* layout)
{
    output_ = addOutput<connection_types::CvMatMessage>("Image");

    cap_.open(0);
}


void Camera::update(int slot)
{
}
