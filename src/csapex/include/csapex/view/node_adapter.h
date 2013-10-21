#ifndef NODE_ADAPTER_H
#define NODE_ADAPTER_H

/// SYSTEM
#include <QLayout>

namespace csapex
{

class NodeAdapter
{
public:
    virtual ~NodeAdapter();

    virtual void fill(QBoxLayout* layout);
    virtual void updateDynamicGui(QBoxLayout* layout);
};

}

#endif // NODE_ADAPTER_H
