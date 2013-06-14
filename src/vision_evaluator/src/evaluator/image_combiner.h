#ifndef IMAGE_COMBINER_H
#define IMAGE_COMBINER_H

/// PROJECT
#include <designer/boxed_object.h>

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <QRadioButton>
#include <QBoxLayout>
#include <QFrame>
#include <opencv2/opencv.hpp>

namespace vision_evaluator
{

class ImageCombiner : public BoxedObject
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<ImageCombiner> Ptr;

public:
    virtual ~ImageCombiner();

    virtual cv::Mat combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2) = 0;

    virtual void update_gui(QFrame* additional_holder) {}

    virtual void mousePressEvent(QMouseEvent* event) {}
    virtual void mouseMoveEvent(QMouseEvent* event) {}
    virtual void wheelEvent(QWheelEvent* event) {}
    virtual void keyEvent(QKeyEvent* event) {}

protected:
    virtual void fill(QBoxLayout* layout);
    virtual void insert(QBoxLayout*);
};

} /// NAMESPACE

#endif // IMAGE_COMBINER_H
