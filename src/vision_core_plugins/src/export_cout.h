#ifndef EXPORT_COUT_H
#define EXPORT_COUT_H

/// PROJECT
#include <designer/boxed_object.h>

namespace vision_evaluator {

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
