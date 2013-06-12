/*
 * image_combiner_manager.h
 *
 *  Created on: Apr 2, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef IMAGE_COMBINER_MANAGER_H
#define IMAGE_COMBINER_MANAGER_H

/// PROJECT
#include <utils/plugin_manager.hpp>

/// COMPONENT
#include "image_combiner.h"
#include "registration.hpp"

#define REGISTER_COMBINER(class_name)\
    REGISTER_GENERIC(ImageCombinerManager, class_name)

namespace vision_evaluator
{
class ImageCombinerManager : public ImageCombiner, public PluginManager<ImageCombiner>
{
    Q_OBJECT

public:
    ImageCombinerManager();
    virtual ~ImageCombinerManager();
    virtual void insert(QBoxLayout*);

    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);
    virtual void wheelEvent(QWheelEvent* event);
    virtual void keyEvent(QKeyEvent* event);

    void setAdditionalHolder(QFrame* frame);

protected Q_SLOTS:
    void update();
    virtual cv::Mat combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2);

Q_SIGNALS:
    void combinerInstalled();
    void combinerDeinstalled();
    void nextImageRequest();
    void outputMat(cv::Mat, cv::Mat);
    void display_request(const QSharedPointer<QImage>);

private:
    ImageCombiner::Ptr active;
    std::vector<QRadioButton*> buttons;
    QFrame* additional_holder_;
};

}

#endif // IMAGE_COMBINER_MANAGER_H
