#ifndef EXPORT_COUT_H
#define EXPORT_COUT_H

/// PROJECT
#include <csapex/boxed_object.h>

namespace csapex {

class ExportCout : public BoxedObject
{
    Q_OBJECT

public:
    ExportCout();

    virtual void fill(QBoxLayout* layout);
    virtual void messageArrived(ConnectorIn* source);

private:
    ConnectorIn* connector_;
};

}

#endif // EXPORT_COUT_H
