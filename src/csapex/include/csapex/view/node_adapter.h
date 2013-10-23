#ifndef NODE_ADAPTER_H
#define NODE_ADAPTER_H

/// SYSTEM
#include <QLayout>
#include <boost/shared_ptr.hpp>

namespace csapex
{

class NodeAdapter
{
public:
    typedef boost::shared_ptr<NodeAdapter> Ptr;

public:
    virtual ~NodeAdapter();

    virtual void fill(QBoxLayout* layout);
    virtual void updateDynamicGui(QBoxLayout* layout);
};

}

#endif // NODE_ADAPTER_H
