#ifndef NODE_ADAPTER_BUILDER_H
#define NODE_ADAPTER_BUILDER_H

/// COMPONENT
#include <csapex_qt/export.h>

/// PROJECT
#include <csapex/view/view_fwd.h>
#include <csapex/model/model_fwd.h>
#include <csapex/utility/type.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <string>
#include <memory>
#include <sstream>

namespace csapex
{
class CSAPEX_QT_EXPORT NodeAdapterBuilder
{
public:
    typedef std::shared_ptr<NodeAdapterBuilder> Ptr;

public:
    virtual ~NodeAdapterBuilder();

    virtual bool isProxy() const = 0;

    void setType(const std::string& type);
    std::string getType() const;

    virtual std::string getWrappedType() const = 0;

    virtual NodeAdapterPtr build(NodeFacadePtr node, NodeBox* parent) const = 0;

private:
    std::string type_;
};

namespace impl
{
NodePtr getNode(const NodeFacadePtr& nf);
NodeFacadeProxyPtr castToProxy(const NodeFacadePtr& facade);
NodeFacadeImplementationPtr castToImplementation(const NodeFacadePtr& facade);

template <typename Adapter, typename Adaptee>
class LocalAdapterBuilder : public csapex::NodeAdapterBuilder
{
public:
    bool isProxy() const override
    {
        return false;
    }
    std::string getWrappedType() const override
    {
        return csapex::type2name<Adaptee>();
    }
    csapex::NodeAdapterPtr build(csapex::NodeFacadePtr facade, csapex::NodeBox* parent) const override
    {
        std::weak_ptr<Adaptee> adaptee = std::dynamic_pointer_cast<Adaptee>(getNode(facade));
        apex_assert_hard_msg(adaptee.lock(), std::string("The adapter ") + getWrappedType() + " is used in a remote setting");
        return std::make_shared<Adapter>(castToImplementation(facade), parent, adaptee);
    }
};

template <typename Adapter>
class ProxyAdapterBuilder : public csapex::NodeAdapterBuilder
{
public:
    ProxyAdapterBuilder(const std::string& wrapped_type) : wrapped_type_(wrapped_type)
    {
    }

    bool isProxy() const override
    {
        return true;
    }
    std::string getWrappedType() const override
    {
        return wrapped_type_;
    }
    csapex::NodeAdapterPtr build(csapex::NodeFacadePtr facade, csapex::NodeBox* parent) const override
    {
        return std::make_shared<Adapter>(facade, parent);
    }

private:
    std::string wrapped_type_;
};
}  // namespace impl

}  // namespace csapex

#endif  // NODE_ADAPTER_BUILDER_H
