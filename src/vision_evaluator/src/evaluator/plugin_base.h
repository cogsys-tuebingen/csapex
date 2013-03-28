#ifndef PLUGIN_BASE_H
#define PLUGIN_BASE_H

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <boost/signals2.hpp>
#include <opencv2/opencv.hpp>
#include <QObject>
#include <QLayout>
#include <QToolBox>

namespace vision_evaluator
{

/// FORWARD DECLARATION
class PluginQueue;

class PluginBase : public QObject
{
    Q_OBJECT

protected:
    PluginBase(const std::string& name);

public:
    typedef boost::shared_ptr<PluginBase> PluginPtr;
    typedef boost::function<bool(PluginBase::PluginPtr)> Selector;

public:
    virtual ~PluginBase();

    static void init(int argc, char** argv) {}

    virtual void refresh() {}
    virtual void insert(QBoxLayout*) = 0;
    virtual void filter(cv::Mat img, cv::Mat mask);
    virtual cv::Mat combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2);
    std::string getName();

    virtual PluginPtr metaInstance();
    virtual void init_gui(QToolBox* toolbox) {}
    virtual void setQueue(PluginQueue* queue);

    virtual void mousePressEvent(QMouseEvent* event) {}
    virtual void mouseMoveEvent(QMouseEvent* event) {}
    virtual void wheelEvent(QWheelEvent* event) {}
    virtual void keyEvent(QKeyEvent* event) {}

Q_SIGNALS:
    void plugin_changed();

protected:
    std::string name_;
    PluginQueue* queue_;
};

} /// NAMESPACE

#endif // PLUGIN_BASE_H
