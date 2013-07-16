#ifndef WORKER_H
#define WORKER_H

/// PROJECT
#include "boxed_object.h"

/// SYSTEM
#include <QObject>

namespace vision_evaluator {

class Worker : public QObject
{
    Q_OBJECT

public:
    Worker();

    virtual BoxedObject* getParent() = 0;
};

}

#endif // WORKER_H
