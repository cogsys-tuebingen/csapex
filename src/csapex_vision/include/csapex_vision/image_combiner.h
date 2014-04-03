#ifndef IMAGE_COMBINER_H
#define IMAGE_COMBINER_H

/// PROJECT
#include <csapex/model/boxed_object.h>

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <QRadioButton>
#include <QBoxLayout>
#include <QFrame>
#include <opencv2/opencv.hpp>

namespace csapex
{

class ConnectorIn;
class ConnectorOut;

class ImageCombiner : public BoxedObject
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<ImageCombiner> Ptr;

public:
    ImageCombiner(const UUID& uuid = UUID::NONE);
    virtual ~ImageCombiner();

    virtual cv::Mat combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2) = 0;

    virtual QIcon getIcon() const;

    virtual void update_gui(QFrame* /*additional_holder*/) {}

    virtual void mousePressEvent(QMouseEvent* /*event*/) {}
    virtual void mouseMoveEvent(QMouseEvent* /*event*/) {}
    virtual void wheelEvent(QWheelEvent* /*event*/) {}
    virtual void keyEvent(QKeyEvent* /*event*/) {}

private Q_SLOTS:
    void messageArrived(ConnectorIn* source);
    void process();

protected:
    virtual void fill(QBoxLayout* layout);
    virtual void insert(QBoxLayout*);


    ConnectorIn* input_img_a_;
    ConnectorIn* input_mask_a_;

    ConnectorIn* input_img_b_;
    ConnectorIn* input_mask_b_;

    ConnectorOut* output_img_;

    bool has_img_a;
    bool has_mask_a;
    bool has_img_b;
    bool has_mask_b;
};

} /// NAMESPACE

#endif // IMAGE_COMBINER_H
