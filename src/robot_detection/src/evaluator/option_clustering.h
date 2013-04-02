/*
 * option_clustering.h
 *
 *  Created on: Feb 22, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef OPTION_CLUSTERING_H
#define OPTION_CLUSTERING_H

/// COMPONENT
#include <vision_evaluator/option.h>

/// HEADER
#include "option_clustering.h"

/// SYSTEM
#include <QSlider>

class ClusteringOptions : public vision_evaluator::Option
{
    Q_OBJECT

    ClusteringOptions();
    virtual void insert(QBoxLayout* layout);

private Q_SLOTS:
    void updateSliders();
    void updateThreshold(int value);
    void update();

public:
    static int k;
    static int min_cluster_size;
    static int scaling;

private:
    QSlider* k_slider;
    QSlider* threshold_slider;
    QSlider* min_slider;
    QSlider* scaling_slider;
};

#endif // OPTION_CLUSTERING_H
