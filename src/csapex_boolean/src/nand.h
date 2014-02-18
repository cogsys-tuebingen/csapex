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
    virtual void process();

private:
    ConnectorIn* in_a;
    ConnectorIn* in_b;
    ConnectorOut* out;
};

}

}

#endif // NAND_H
