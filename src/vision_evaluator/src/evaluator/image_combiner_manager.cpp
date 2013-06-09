/*
 * image_combiner_manager.cpp
 *
 *  Created on: Apr 2, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// HEADER
#include "image_combiner_manager.h"

/// SYSTEM
#include <vision_evaluator/qt_helper.hpp>
#include <utils/LibUtil/QtCvImageConverter.h>

using namespace vision_evaluator;

ImageCombinerManager::ImageCombinerManager()
    : PluginManager<vision_evaluator::ImageCombiner>("vision_evaluator::ImageCombiner"), additional_holder_(NULL)
{
}

void ImageCombinerManager::insert(QBoxLayout* layout)
{
    if(!pluginsLoaded()) {
        reload();
    }

    for(std::map<std::string, ImageCombinerManager::Constructor>::const_iterator it = availableClasses().begin(); it != availableClasses().end(); ++it) {
        ImageCombiner::Ptr combiner = (it->second)();

        QRadioButton* btn = new QRadioButton(combiner->getName().c_str());
        layout->addWidget(btn);
        buttons.push_back(btn);

        QObject::connect(btn, SIGNAL(toggled(bool)), this, SLOT(update()));
    }
}

void ImageCombinerManager::setAdditionalHolder(QFrame* frame)
{
    additional_holder_ = frame;
}

void ImageCombinerManager::update()
{
    if(additional_holder_->layout()) {
        QtHelper::clearLayout(additional_holder_->layout());
        delete additional_holder_->layout();
    }

    int i = 0;
    for(std::vector<QRadioButton*>::iterator it = buttons.begin(); it != buttons.end(); ++it) {
        QRadioButton* rb = *it;

        if(rb->isChecked()) {
            active = availableClasses(i)();

            assert(additional_holder_);
            active->update_gui(additional_holder_);
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
