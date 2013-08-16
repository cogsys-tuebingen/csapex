#ifndef CSAPEX_TESTSUITE_H
#define CSAPEX_TESTSUITE_H

/// PROJECT
#include <csapex/box.h>
#include <csapex/boxed_object.h>
#include <csapex/connector_out.h>

/// SYSTEM
#include <QObject>

namespace csapex {

class Dummy : public BoxedObject
{
    Q_OBJECT

public:
    virtual void fill(QBoxLayout*)
    {

    }

public Q_SLOTS:
    virtual void messageArrived(ConnectorIn*)
    {

    }
};

}


#endif // CSAPEX_TESTSUITE_H
