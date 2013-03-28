#ifndef IMAGE_COMBINER_H
#define IMAGE_COMBINER_H

/// COMPONENT
#include "plugin.h"

/// SYSTEM
#include <QRadioButton>

#define REGISTER_IMAGE_COMBINER(type) \
    REGISTER_PLUGIN(ImageCombiner, type)

namespace vision_evaluator
{

class ImageCombiner : public Plugin<ImageCombiner>
{
    Q_OBJECT

public:
    static PluginPtr createMetaInstance();
    virtual PluginPtr metaInstance();

public:
    virtual void setQueue(PluginQueue* queue);
    virtual cv::Mat combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2);


    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);
    virtual void wheelEvent(QWheelEvent* event);
    virtual void keyEvent(QKeyEvent* event);

protected:
    ImageCombiner(const std::string& label);
    virtual void insert(QBoxLayout*);

protected Q_SLOTS:
    void update();

Q_SIGNALS:
    void combinerInstalled();
    void combinerDeinstalled();

private:
    ImageCombiner::TypePtr active;
    std::vector<QRadioButton*> buttons;
};

} /// NAMESPACE

#endif // IMAGE_COMBINER_H
