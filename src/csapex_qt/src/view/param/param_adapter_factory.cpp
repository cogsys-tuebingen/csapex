/// HEADER
#include <csapex/view/param/param_adapter_factory.h>

/// COMPONENT
#include <csapex/model/node_facade.h>
#include <csapex/view/param/param_adapter.h>
#include <csapex/view/node/node_adapter.h>
#include <csapex/view/node/default_node_adapter.h>
#include <csapex/plugin/plugin_manager.hpp>

/// SYSTEM
#include <qglobal.h>

using namespace csapex;

ParameterAdapterFactory::ParameterAdapterFactory() : locator_(nullptr)
{
    manager_ = std::make_shared<PluginManager<ParameterAdapterBuilder>>("csapex::ParameterAdapterBuilder");
}

ParameterAdapterFactory::~ParameterAdapterFactory()
{
    param_adapter_builders_.clear();
}

void ParameterAdapterFactory::registerParameterAdapterFor(const std::type_info& type, ParameterAdapterBuilderPtr builder)
{
    param_adapter_builders_[type] = builder;
}

void ParameterAdapterFactory::loadPlugins(PluginLocator* locator)
{
    manager_->load(locator);

    for (const auto& pair : manager_->getConstructors()) {
        try {
            ParameterAdapterBuilderPtr prov(pair.second());
            registerParameterAdapterFor(prov->getWrappedTypeId(), prov);

        } catch (const std::exception& e) {
            std::cerr << "cannot load parameter adapter" << pair.first << ": " << typeid(e).name() << ", what=" << e.what() << std::endl;
        }
    }
}

bool ParameterAdapterFactory::hasAdapter(const std::type_index& type) const
{
    auto pos = param_adapter_builders_.find(type);
    if (pos != param_adapter_builders_.end()) {
        return pos->second != nullptr;

    } else {
        return false;
    }
}

ParameterAdapter::Ptr ParameterAdapterFactory::makeParameterAdapter(NodeAdapter* adapter, const param::ParameterPtr& parameter_ptr)
{
    DefaultNodeAdapter* def_adapter = dynamic_cast<DefaultNodeAdapter*>(adapter);
    if (!def_adapter) {
        throw std::logic_error("Parameter adapters are currently only supported for default node adapters and their descendants.");
    }
    const auto& parameter = *parameter_ptr;
    auto pos = param_adapter_builders_.find(typeid(parameter));
    if (pos != param_adapter_builders_.end()) {
        ParameterAdapterBuilder& builder = *pos->second;
        return builder.build(def_adapter, parameter_ptr);

    } else {
        throw std::logic_error("Tried to instantiate an unsupported parameter adapter.");
    }
}
