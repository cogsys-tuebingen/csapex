#ifndef FILTER_H
#define FILTER_H

/// PROJECT
#include <designer/boxed_object.h>

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <QObject>
#include <QBoxLayout>

namespace vision_evaluator
{

class ConnectorIn;
class ConnectorOut;

class Filter : public BoxedObject
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<Filter> Ptr;

public:
    virtual ~Filter();

Q_SIGNALS:
    void filter_changed();

public Q_SLOTS:
    virtual void filter(cv::Mat& img, cv::Mat& mask) = 0;
    virtual void fill(QBoxLayout* layout);
    virtual void insert(QBoxLayout* parent) = 0;

private Q_SLOTS:
    void messageArrived(ConnectorIn* source);

protected:
    Filter();
    virtual bool usesMask();

protected:
    ConnectorIn* input_img_;
    ConnectorIn* input_mask_;

    ConnectorOut* output_img_;
    ConnectorOut* output_mask_;

    bool has_img;
    bool has_mask;
    long guard;
};

} /// NAMESPACE

#endif // FILTER_H
