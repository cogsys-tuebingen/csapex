#ifndef PLUGIN_QUEUE_H
#define PLUGIN_QUEUE_H

/// COMPONENT
#include "plugin_base.h"

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <boost/signals2.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <QImage>
#include <QSharedPointer>
#include <QToolBox>

namespace vision_evaluator
{

/// FORWARD DECLARATION
class Viewer;

class PluginQueue : public QObject
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<PluginQueue> Ptr;

public:
    PluginQueue(QWidget* parent, QToolBox* toolbox);

    void registerPlugin(PluginBase::PluginPtr plugin);
    void addMeta(PluginBase::PluginPtr plugin);


    template <class Container, class PluginType>
    PluginType* getInstance() {
        for(unsigned i = 0; i < meta.size(); ++i) {
            if(typeid(*meta[i].get()) == typeid(PluginType)) {
                return dynamic_cast<PluginType*>(meta[i].get());

            } else if(typeid(*meta[i].get()) == typeid(Container)) {
                PluginType* p = ((Container*) meta[i].get())->getInstance<PluginType>();
                if(p) {
                    return p;
                }
            }
        }
        return NULL;
    }

public Q_SLOTS:
    void input(cv::Mat input, cv::Mat mask);
    void combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2);

    void mousePressEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
    void wheelEvent(QWheelEvent* event);
    void keyEvent(QKeyEvent* event);

Q_SIGNALS:
    void outputMat(cv::Mat, cv::Mat);
    void display_request(const QSharedPointer<QImage>);
    void plugin_changed();
    void nextImageRequest();
    void combinerInstalled();
    void combinerDeinstalled();

public:
    Viewer* viewer;

protected:
    void refresh();

private:
    QBoxLayout* layout;
    QWidget* parent;
    QToolBox* toolbox_;
    std::vector<PluginBase::PluginPtr> meta;
};

} /// NAMESPACE

#endif // PLUGIN_QUEUE_H
