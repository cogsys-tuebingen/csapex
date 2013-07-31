#ifndef FILTER_TOOL_DETECTION_H
#define FILTER_TOOL_DETECTION_H

/// COMPONENT
#include <csapex_vision/filter.h>

/// SYSTEM
#include <QSlider>

namespace cip104
{

class LKTracking : public csapex::Filter
{
    Q_OBJECT

public:
    LKTracking();

public:
    virtual void filter(cv::Mat& img, cv::Mat& mask);
    virtual void insert(QBoxLayout* layout);

public Q_SLOTS:
    void update();
    void reset();

protected:
    void init(cv::Mat &gray, cv::Mat &mask);

private:
    bool init_;

    std::vector<cv::Point2f> points[2];
    int max_count;
    cv::TermCriteria termcrit;
    cv::Size subPixWinSize;
    cv::Size winSize;

    cv::Mat gray;
    cv::Mat prevGray;
};

}

#endif // FILTER_TOOL_DETECTION_H
