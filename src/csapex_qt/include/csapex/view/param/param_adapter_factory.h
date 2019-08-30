#ifndef PARAM_ADAPTER_FACTORY_H
#define PARAM_ADAPTER_FACTORY_H

/// COMPONENT
#include <csapex_qt/export.h>
#include <csapex/core/core_fwd.h>
#include <csapex/model/model_fwd.h>
#include <csapex/plugin/plugin_fwd.h>
#include <csapex/view/param/param_adapter_builder.h>
#include <csapex/param/param_fwd.h>

/// PROJECT
#include <csapex/utility/singleton.hpp>

/// SYSTEM
#include <map>
#include <typeindex>

namespace csapex
{
class CSAPEX_QT_EXPORT ParameterAdapterFactory : public Singleton<ParameterAdapterFactory>
{
public:
    typedef std::shared_ptr<ParameterAdapterFactory> Ptr;

public:
    ParameterAdapterFactory();
    ~ParameterAdapterFactory();

    void loadPlugins(PluginLocator* locator);

    bool hasAdapter(const std::type_index& type) const;
    ParameterAdapterPtr makeParameterAdapter(NodeAdapter* adapter, const param::ParameterPtr& parameter);

    template <typename Builder, typename Parameter>
    static void registerParameterAdapter()
    {
        ParameterAdapterFactory::instance().registerParameterAdapterFor(typeid(Parameter), std::make_shared<Builder>());
    }

    void registerParameterAdapterFor(const std::type_info& type, ParameterAdapterBuilderPtr builder);

protected:
    std::shared_ptr<PluginManager<ParameterAdapterBuilder>> manager_;
    PluginLocator* locator_;
    std::map<std::type_index, ParameterAdapterBuilder::Ptr> param_adapter_builders_;
};

template <typename Builder, typename Parameter>
struct ParameterAdapterRegistered
{
    ParameterAdapterRegistered()
    {
        csapex::ParameterAdapterFactory::registerParameterAdapter<Builder, Parameter>();
    }
};

}  // namespace csapex

#endif  // PARAM_ADAPTER_FACTORY_H
