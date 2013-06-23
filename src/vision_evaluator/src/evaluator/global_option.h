/*
 * global_option.h
 *
 *  Created on: Mar 28, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef GLOBAL_OPTION_H
#define GLOBAL_OPTION_H

/// PROJECT
#include <designer/memento.h>

/// SYSTEM
#include <QBoxLayout>
#include <boost/shared_ptr.hpp>

namespace vision_evaluator
{

class GlobalOption : public QObject
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<GlobalOption> Ptr;

public:
    GlobalOption();

    virtual void insert(QBoxLayout* layout) = 0;

    virtual void setName(const std::string& name);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

protected:
    std::string name_;
};

} /// NAMESPACE

#endif // GLOBAL_OPTION_H
