#ifndef INDICATOR_H
#define INDICATOR_H

/// HEADER
#include <csapex/model/boxed_object.h>

/// SYSTEM
#include <QCheckBox>

namespace csapex {

namespace boolean {

class Indicator : public BoxedObject
{
    Q_OBJECT

public:
    Indicator();

public:
    virtual void fill(QBoxLayout* layout);

public Q_SLOTS:
    virtual void allConnectorsArrived();

private:
    ConnectorIn* in;

    QCheckBox* indicator_;
};

}

}

#endif // INDICATOR_H
