/*
 * option_clustering.cpp
 *
 *  Created on: Feb 22, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// HEADER
#include "option_clustering.h"

REGISTER_OPTION(ClusteringOptions)

int ClusteringOptions::k = 4;

ClusteringOptions::ClusteringOptions()
    : Option("Clustering Options")
{
}

Option::TypePtr ClusteringOptions::createInstance(CONSTRUCTOR_MODE mode)
{
    return Option::TypePtr(new ClusteringOptions);
}

void ClusteringOptions::insert(QLayout* layout)
{
    k_slider = new QSlider(Qt::Horizontal);
    k_slider->setMinimum(1);
    k_slider->setMaximum(20);
    k_slider->setValue(k);
    layout->addWidget(k_slider);

    QObject::connect(k_slider, SIGNAL(valueChanged(int)), this, SLOT(update(int)));
}

void ClusteringOptions::update()
{
    Q_EMIT plugin_changed();
}

void ClusteringOptions::update(int value)
{
    k = value;
    WARN("k=" << k);
    Q_EMIT plugin_changed();
}

