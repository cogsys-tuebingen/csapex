/*
 * option_clustering.h
 *
 *  Created on: Feb 22, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef OPTION_CLUSTERING_H
#define OPTION_CLUSTERING_H

/// COMPONENT
#include "option.h"

/// HEADER
#include "option_clustering.h"

/// SYSTEM
#include <QSlider>

class ClusteringOptions : public Option
{
    Q_OBJECT

    ClusteringOptions();
    virtual void insert(QLayout* layout);

public:
    static Option::TypePtr createInstance(CONSTRUCTOR_MODE mode);

private Q_SLOTS:
    void update(int value);
    void update();

public:
    static int k;

private:
    QSlider* k_slider;
};

#endif // OPTION_CLUSTERING_H
