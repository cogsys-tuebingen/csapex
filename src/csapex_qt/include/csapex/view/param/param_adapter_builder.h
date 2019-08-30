#ifndef PARAM_ADAPTER_BUILDER_H
#define PARAM_ADAPTER_BUILDER_H

/// COMPONENT
#include <csapex_qt/export.h>

/// PROJECT
#include <csapex/view/view_fwd.h>
#include <csapex/view/node/default_node_adapter.h>
#include <csapex/model/model_fwd.h>
#include <csapex/param/param_fwd.h>
#include <csapex/utility/type.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <string>
#include <memory>
#include <sstream>

namespace csapex
{
class CSAPEX_QT_EXPORT ParameterAdapterBuilder
{
public:
    typedef std::shared_ptr<ParameterAdapterBuilder> Ptr;

public:
    virtual ~ParameterAdapterBuilder();

    virtual std::string getWrappedType() const = 0;
    virtual const std::type_info& getWrappedTypeId() const = 0;

    virtual ParameterAdapterPtr build(DefaultNodeAdapter* adapter, const param::ParameterPtr& parameter) const = 0;

private:
    std::string type_;
};

namespace impl
{
template <typename Adapter, typename Adaptee>
class ParameterAdapterBuilderImplementation : public csapex::ParameterAdapterBuilder
{
public:
    virtual std::string getWrappedType() const
    {
        return csapex::type2name<Adaptee>();
    }
    virtual csapex::ParameterAdapterPtr build(DefaultNodeAdapter* node_adapter, const param::ParameterPtr& parameter) const
    {
        ParameterAdapterPtr adapter(std::make_shared<Adapter>(std::dynamic_pointer_cast<Adaptee>(parameter)));
        node_adapter->addParameterAdapter(parameter, adapter);
        return {};
    }
};

}  // namespace impl

}  // namespace csapex

#endif  // PARAM_ADAPTER_BUILDER_H
