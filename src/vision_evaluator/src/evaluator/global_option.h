/*
 * global_option.h
 *
 *  Created on: Mar 28, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef GLOBAL_OPTION_H
#define GLOBAL_OPTION_H

/// SYSTEM
#include <QBoxLayout>

namespace vision_evaluator
{

class GlobalOption : public QObject
{
    Q_OBJECT

public:
    GlobalOption();

    virtual void insert(QBoxLayout* layout) = 0;
};

} /// NAMESPACE

#endif // GLOBAL_OPTION_H
