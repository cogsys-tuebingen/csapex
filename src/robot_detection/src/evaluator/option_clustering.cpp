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
#include <QLabel>

int ClusteringOptions::k = 8;
int ClusteringOptions::scaling = 12;
int ClusteringOptions::min_cluster_size = 4;

ClusteringOptions::ClusteringOptions()
{
}

namespace
{
QSlider* makeSlider(QBoxLayout* layout, const std::string& name, int def, int min, int max)
{
    QHBoxLayout* internal_layout = new QHBoxLayout;

    QSlider* slider = new QSlider(Qt::Horizontal);
    slider->setMinimum(min);
    slider->setMaximum(max);
    slider->setValue(def);

    internal_layout->addWidget(new QLabel(name.c_str()));
    internal_layout->addWidget(slider);

    layout->addLayout(internal_layout);

    return slider;
}
}

void ClusteringOptions::insert(QBoxLayout* layout)
{
    k_slider = makeSlider(layout, "k Clusters", k, 1,32);
    scaling_slider = makeSlider(layout, "scaling", scaling, 1,32);
    threshold_slider = makeSlider(layout, "threshold", Config::getGlobal().matcher_threshold * 100, 1,100);
    min_slider = makeSlider(layout, "min cluster size", 4, 1,32);

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
    Config config = Config::getGlobal();

    double t = value / 100.0;

    if(config.matcher_threshold == t) {
        return;
    }

    config.matcher_threshold = t;
    config.replaceGlobal();

    plugin_changed();
}

