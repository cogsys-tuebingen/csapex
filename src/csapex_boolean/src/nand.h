#ifndef NAND_H
#define NAND_H

/// HEADER
#include <csapex/model/boxed_object.h>

namespace csapex {

namespace boolean {

class NAND : public BoxedObject
{
    Q_OBJECT

public:
    NAND();

public:
    virtual void fill(QBoxLayout* layout);

public Q_SLOTS:
    virtual void messageArrived(ConnectorIn* source);

private:
    ConnectorIn* in_a;
    ConnectorIn* in_b;
    ConnectorOut* out;

    bool has_a;
    bool has_b;

};

}

}

#endif // NAND_H
