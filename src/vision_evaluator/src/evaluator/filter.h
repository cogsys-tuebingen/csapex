#ifndef FILTER_H
#define FILTER_H

/// COMPONENT
#include "plugin.h"

/// SYSTEM
#include <opencv2/opencv.hpp>
#include <QSlider>

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

    QSlider* makeSlider(QBoxLayout* layout, const std::string& name, int def, int min, int max);

private:
    std::string name_;
};

} /// NAMESPACE

#endif // FILTER_H
