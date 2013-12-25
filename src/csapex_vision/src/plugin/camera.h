#ifndef CAMERA_H_
#define CAMERA_H_

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

/// HEADER
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>

namespace csapex
{

class Camera : public Node
{
public:
    Camera();

    virtual void tick();
    virtual void setup();

protected:
    void update(param::Parameter *p);

private:
    ConnectorOut* output_;
    cv::VideoCapture cap_;
};

} /// NAMESPACE

#endif // CAMERA_H_
