#ifndef NODE_ADAPTER_BUILDER_H
#define NODE_ADAPTER_BUILDER_H

/// PROJECT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <string>
#include <memory>

namespace csapex
{

class NodeAdapterBuilder
{
public:
    typedef std::shared_ptr<NodeAdapterBuilder> Ptr;

public:
    virtual ~NodeAdapterBuilder();

    void setType(const std::string& type);
    std::string getType() const;

    virtual std::string getWrappedType() const = 0;

    virtual NodeAdapterPtr build(NodeWorker* node, WidgetController* widget_ctrl) const = 0;

private:
    std::string type_;
};

}

#endif // NODE_ADAPTER_BUILDER_H
