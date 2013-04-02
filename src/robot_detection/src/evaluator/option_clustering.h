/*
 * option_clustering.h
 *
 *  Created on: Feb 22, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef OPTION_CLUSTERING_H
#define OPTION_CLUSTERING_H

/// HEADER
#include "option_clustering.h"

/// SYSTEM
#include <QSlider>
#include <QBoxLayout>

class ClusteringOptions : public QObject
{
    Q_OBJECT

    ClusteringOptions();
    virtual void insert(QBoxLayout* layout);

private Q_SLOTS:
    void updateSliders();
    void updateThreshold(int value);
    void update();

Q_SIGNALS:
    void plugin_changed();

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
