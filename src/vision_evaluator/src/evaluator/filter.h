#ifndef FILTER_H
#define FILTER_H

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <QObject>
#include <QBoxLayout>

namespace vision_evaluator
{

class Filter : public QObject
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<Filter> Ptr;

public:
    virtual ~Filter();

Q_SIGNALS:
    void filter_changed();

public Q_SLOTS:
    virtual std::string getName();
    void setName(const std::string& name);

    virtual void filter(cv::Mat img, cv::Mat mask) = 0;
    virtual void insert(QBoxLayout* parent) {}

protected:
    Filter();

private:
    std::string name_;
};

} /// NAMESPACE

#endif // FILTER_H
