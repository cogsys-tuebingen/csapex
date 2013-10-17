#ifndef CAMERA_H_
#define CAMERA_H_

/// PROJECT
#include <csapex/model/boxed_object.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex
{

class Camera : public BoxedObject
{
    Q_OBJECT

public:
    Camera();

    virtual void tick();
    virtual void fill(QBoxLayout* layout);

private Q_SLOTS:
    void update(int slot);

private:
    ConnectorOut* output_;
    cv::VideoCapture cap_;
};

} /// NAMESPACE

#endif // CAMERA_H_
