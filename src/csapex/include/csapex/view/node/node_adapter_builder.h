#ifndef NODE_ADAPTER_BUILDER_H
#define NODE_ADAPTER_BUILDER_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>

/// PROJECT
#include <csapex/view/view_fwd.h>
#include <csapex/model/model_fwd.h>

/// SYSTEM
#include <string>
#include <memory>

namespace csapex
{

class CSAPEX_QT_EXPORT NodeAdapterBuilder
{
public:
    typedef std::shared_ptr<NodeAdapterBuilder> Ptr;

public:
    virtual ~NodeAdapterBuilder();

    void setType(const std::string& type);
    std::string getType() const;

    virtual std::string getWrappedType() const = 0;

    virtual NodeAdapterPtr build(NodeHandlePtr node, NodeBox* parent) const = 0;

private:
    std::string type_;
};

}

#endif // NODE_ADAPTER_BUILDER_H
