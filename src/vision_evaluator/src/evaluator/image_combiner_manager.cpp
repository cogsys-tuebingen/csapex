/*
 * image_combiner_manager.cpp
 *
 *  Created on: Apr 2, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// HEADER
#include "image_combiner_manager.h"

/// SYSTEM
#include <utils/LibUtil/QtCvImageConverter.h>

using namespace vision_evaluator;

ImageCombinerManager::ImageCombinerManager()
    : GenericManager<vision_evaluator::ImageCombiner>("vision_evaluator::ImageCombiner")
{
}

void ImageCombinerManager::insert(QBoxLayout* layout)
{
    if(!plugins_loaded_) {
        reload();
    }

    for(std::vector<ImageCombinerManager::Constructor>::iterator it = available_classes.begin(); it != available_classes.end(); ++it) {
        ImageCombiner::Ptr combiner = (*it)();

        QRadioButton* btn = new QRadioButton(combiner->getName().c_str());
        layout->addWidget(btn);
        buttons.push_back(btn);

        QObject::connect(btn, SIGNAL(toggled(bool)), this, SLOT(update()));
    }
}

void ImageCombinerManager::update()
{
    int i = 0;
    for(std::vector<QRadioButton*>::iterator it = buttons.begin(); it != buttons.end(); ++it) {
        QRadioButton* rb = *it;

        if(rb->isChecked()) {
            active = available_classes[i]();
//            active->init_gui(NULL);
//            active->setQueue(queue_);

            Q_EMIT combinerInstalled();

            return;
        }

        ++i;
    }

    Q_EMIT combinerDeinstalled();
}

cv::Mat ImageCombinerManager::combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2)
{
    if(active.get() == NULL) {
        return img1;
    } else {
        cv::Mat result = active->combine(img1, mask1, img2, mask2);

        Q_EMIT outputMat(result, cv::Mat());
        Q_EMIT display_request(QtCvImageConverter::Converter<QImage, QSharedPointer>::mat2QImage(result));
        Q_EMIT nextImageRequest();

        return result;
    }
}


void ImageCombinerManager::mousePressEvent(QMouseEvent* event)
{
    if(active != NULL) {
        active->mousePressEvent(event);
    }
}

void ImageCombinerManager::mouseMoveEvent(QMouseEvent* event)
{
    if(active != NULL) {
        active->mouseMoveEvent(event);
    }
}

void ImageCombinerManager::wheelEvent(QWheelEvent* event)
{
    if(active != NULL) {
        active->wheelEvent(event);
    }
}

void ImageCombinerManager::keyEvent(QKeyEvent* event)
{
    if(active != NULL) {
        active->keyEvent(event);
    }
}
