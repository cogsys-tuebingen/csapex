#ifndef VIEWER_H
#define VIEWER_H

/// COMPONENT
#include "image_provider.h"
#include "plugin_queue.h"

/// SYSTEM
#include <boost/signals2.hpp>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>
#include <QFrame>
#include <QTimer>
#include <string>

namespace vision_evaluator
{

class Viewer : public QObject
{
    Q_OBJECT

public:
    Viewer(QFrame* additional_holder, PluginQueue::Ptr queue);
    ~Viewer();

    void setProvider(QSharedPointer<ImageProvider> provider);

public Q_SLOTS:
    void update_gui();
    void handle(const std::string& path);
    void provideNextImage();
    void repeat();
    void tick();
    void iteration();
    void set_fps(int fps);
    void setOneShotMode(bool mode);

Q_SIGNALS:
    void image_provided(cv::Mat img, cv::Mat mask);
    void update_request();
    void gui_updated();

public:
    std::string last_path_;

    QTimer* ticker_;
    QFrame* additional_holder_;

    QSharedPointer<ImageProvider> provider_;

    void clearLayout();

private:
    int fps;
    bool one_shot_mode;
    int requests;
    PluginQueue::Ptr queue;
};

} /// NAMESPACE

#endif // VIEWER_H
