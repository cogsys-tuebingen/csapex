#ifndef OPTION_H
#define OPTION_H

/// COMPONENT
#include "plugin.h"

/// SYSTEM
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <QWidget>
#include <QLayout>

#define REGISTER_OPTION(type) \
    REGISTER_PLUGIN(Option, type)

namespace vision_evaluator
{

class Option : public Plugin<Option>
{
    Q_OBJECT

public:
    static PluginPtr createMetaInstance();
    virtual PluginPtr metaInstance();

    virtual ~Option();

protected:
    Option(const std::string& label);

public:
    virtual void refresh();
    virtual void insert(QBoxLayout* layout);

    template <class PluginType>
    PluginType* getInstance() {
        for(unsigned i = 0; i < active_options.size(); ++i) {
            if(typeid(*active_options[i].get()) == typeid(PluginType)) {
                return dynamic_cast<PluginType*>(active_options[i].get());
            }
        }
        return NULL;
    }

private:
    std::vector<Option::PluginPtr> active_options;
    std::vector<Option::PluginPtr> displayed_options;
};

} /// NAMESPACE

#endif // OPTION_H
