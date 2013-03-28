/// HEADER
#include "plugin_queue.h"
#include "image_combiner.h"

/// COMPONENT
#include "plugin_manager.h"

/// SYSTEM
#include <QWidget>
#include <utils/LibUtil/QtCvImageConverter.h>
#include <QPointer>

using namespace vision_evaluator;

PluginQueue::PluginQueue(QWidget* parent, QToolBox* toolbox)
    : parent(parent), toolbox_(toolbox)
{
    refresh();
}

void PluginQueue::registerPlugin(PluginBase::PluginPtr plugin)
{
    plugin->setQueue(this);
}

void PluginQueue::addMeta(PluginBase::PluginPtr plugin)
{
    QObject::connect(plugin.get(), SIGNAL(plugin_changed()), this, SIGNAL(plugin_changed()));
    meta.push_back(plugin);
    if(toolbox_ != NULL) {
        plugin->init_gui(toolbox_);
    }
    plugin->refresh();

    ImageCombiner* combiner = dynamic_cast<ImageCombiner*>(plugin.get());
    if(combiner) {
        QObject::connect(combiner, SIGNAL(combinerInstalled()), this, SIGNAL(combinerInstalled()));
        QObject::connect(combiner, SIGNAL(combinerDeinstalled()), this, SIGNAL(combinerDeinstalled()));
    }
}

void PluginQueue::input(cv::Mat input, cv::Mat mask)
{
    if(mask.empty()) {
        mask = cv::Mat(input.rows, input.cols, CV_8UC1, cv::Scalar::all(255));
    }

    if(!meta.empty()) {
        for(unsigned i = 0; i < meta.size(); ++i) {
            meta[i]->filter(input, mask);
        }
    }
    Q_EMIT outputMat(input, mask);
    Q_EMIT display_request(QtCvImageConverter::Converter<QImage, QSharedPointer>::mat2QImage(input));
}

void PluginQueue::mouseMoveEvent(QMouseEvent* event)
{
    for(unsigned i = 0; i < meta.size(); ++i) {
        meta[i]->mouseMoveEvent(event);
    }
}

void PluginQueue::mousePressEvent(QMouseEvent* event)
{
    for(unsigned i = 0; i < meta.size(); ++i) {
        meta[i]->mousePressEvent(event);
    }
}

void PluginQueue::wheelEvent(QWheelEvent* event)
{
    for(unsigned i = 0; i < meta.size(); ++i) {
        meta[i]->wheelEvent(event);
    }
}

void PluginQueue::keyEvent(QKeyEvent* event)
{
    for(unsigned i = 0; i < meta.size(); ++i) {
        meta[i]->keyEvent(event);
    }
}

void PluginQueue::combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2)
{
    if(!meta.empty()) {
        //        assert(meta.size() == 1);

        for(unsigned i = 0; i < meta.size(); ++i) {
            if(dynamic_cast<ImageCombiner*>(meta[i].get())) {
                cv::Mat tmp = meta[i]->combine(img1, mask1, img2, mask2);
                Q_EMIT outputMat(tmp, cv::Mat());
                Q_EMIT display_request(QtCvImageConverter::Converter<QImage, QSharedPointer>::mat2QImage(tmp));
                Q_EMIT nextImageRequest();
                break;
            }
        }
    }

}

void PluginQueue::refresh()
{
}
