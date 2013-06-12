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

class Filter : public QObject, public BoxedObject
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
    virtual void fill(QBoxLayout *layout);
    virtual void insert(QBoxLayout* parent) = 0;

protected:
    Filter();

protected:
    ConnectorIn* input_;
    ConnectorOut* output_;
};

} /// NAMESPACE

#endif // FILTER_H
