#ifndef EXPORT_COUT_H
#define EXPORT_COUT_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class ExportCout : public Node
{
public:
    ExportCout();

    virtual void setup();
    virtual void process();

private:
    ConnectorIn* connector_;
};

}

#endif // EXPORT_COUT_H
