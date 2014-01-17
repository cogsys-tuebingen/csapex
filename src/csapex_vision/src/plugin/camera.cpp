/// HEADER
#include "camera.h"

/// COMPONENT
#include <csapex_vision/cv_mat_message.h>

#include <utils_param/parameter_factory.h>
/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::Camera, csapex::Node)

using namespace csapex;

Camera::Camera()
    : idx(-1)
{
    addTag(Tag::get("Input"));
    addTag(Tag::get("Vision"));

    addParameter(param::ParameterFactory::declare<int>("device", 0, 5, 0, 1), boost::bind(&Camera::update, this, _1));

    addParameter(param::ParameterFactory::declare<int>("w", 640, 1280, 640, 1), boost::bind(&Camera::update, this, _1));
    addParameter(param::ParameterFactory::declare<int>("h", 480, 800, 480, 1), boost::bind(&Camera::update, this, _1));
}

void Camera::tick()
{
    if(cap_.isOpened()) {
        connection_types::CvMatMessage::Ptr msg(new connection_types::CvMatMessage);
        cap_ >> msg->value;
        output_->publish(msg);
    }
}

void Camera::setup()
{
    Node::setup();
    output_ = addOutput<connection_types::CvMatMessage>("Image");
}


void Camera::update(param::Parameter *p)
{
    setError(false);
    int dev = param<int>("device");

    if(cap_.isOpened()) {
        cap_.release();
    }

    if(!cap_.open(dev)) {
        throw std::runtime_error("cannot open camera with the given id");
    }

    std::cout << "camera settings" << std::endl;
    std::cout << cap_.get(CV_CAP_PROP_FRAME_WIDTH) << " x " << cap_.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;
    cap_.set(CV_CAP_PROP_FRAME_WIDTH, param<int>("w"));
    cap_.set(CV_CAP_PROP_FRAME_HEIGHT, param<int>("h"));
    std::cout << cap_.get(CV_CAP_PROP_FRAME_WIDTH) << " x " << cap_.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;
}
