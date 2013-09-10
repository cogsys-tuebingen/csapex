/*
 * option_clustering.cpp
 *
 *  Created on: Feb 22, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// HEADER
#include "option_clustering.h"

/// PROJECT
#include <config/config.h>

/// SYSTEM
#include <csapex/utility/qt_helper.hpp>
#include <QLabel>


ClusteringOptions::ClusteringOptions()
{
    k = 8;
    scaling = 12;
    min_cluster_size = 4;
}

void ClusteringOptions::insert(QBoxLayout* layout)
{
    k_slider = QtHelper::makeSlider(layout, "k Clusters", k, 1,32);
    scaling_slider = QtHelper::makeSlider(layout, "scaling", scaling, 1,32);
    threshold_slider = QtHelper::makeSlider(layout, "(global) matcher threshold", Config::instance()("matcher_threshold").as<double>() * 100, 1,100);
    min_slider = QtHelper::makeSlider(layout, "min cluster size", 4, 1,32);

    QObject::connect(k_slider, SIGNAL(valueChanged(int)), this, SLOT(updateSliders()));
    QObject::connect(scaling_slider, SIGNAL(valueChanged(int)), this, SLOT(updateSliders()));
    QObject::connect(threshold_slider, SIGNAL(valueChanged(int)), this, SLOT(updateThreshold(int)));
    QObject::connect(min_slider, SIGNAL(valueChanged(int)), this, SLOT(updateSliders()));
}

void ClusteringOptions::update()
{
    Q_EMIT plugin_changed();
}

void ClusteringOptions::updateSliders()
{
    k = k_slider->value();
    min_cluster_size = min_slider->value();
    scaling = scaling_slider->value();
    Q_EMIT plugin_changed();
}


void ClusteringOptions::updateThreshold(int value)
{
    Config config = Config::instance();

    double t = value / 100.0;

    if(config("matcher_threshold").as<double>() == t) {
        return;
    }

    config["matcher_threshold"] = t;
    config.replaceInstance();

    plugin_changed();
}

