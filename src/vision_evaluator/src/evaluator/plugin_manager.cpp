/// HEADER
#include "plugin_manager.h"

/// COMPONENT
#include "plugin.h"

/// SYSTEM
#include <QBoxLayout>

using namespace vision_evaluator;

PluginManager::PluginManager()
    : initiated_(false)
{
}

PluginManager::~PluginManager()
{
}

PluginManager& PluginManager::instance()
{
    static PluginManager instance;
    return instance;
}

void PluginManager::register_meta_plugin(MetaInit init, MetaConstructor meta)
{
    meta_plugin_constructors_.push_back(std::make_pair(init, meta));
}

void PluginManager::init(int argc, char** argv)
{
    for(std::vector<std::pair<MetaInit, MetaConstructor> >::iterator it = meta_plugin_constructors_.begin(); it != meta_plugin_constructors_.end(); ++it) {
        it->first(argc, argv);
    }
    for(std::vector<std::pair<MetaInit, MetaConstructor> >::iterator it = meta_plugin_constructors_.begin(); it != meta_plugin_constructors_.end(); ++it) {
        meta_plugins_.push_back((it->second)());
    }

    initiated_ = true;
}

PluginQueue::Ptr PluginManager::createQueue(QWidget* parent, QToolBox* toolbox, const std::vector<PluginBase::Selector> &selectors,
        bool horizontal)
{
    PluginQueue::Ptr result(new PluginQueue(parent, toolbox));

    QBoxLayout* layout;
    if(horizontal) {
        layout = new QHBoxLayout;
        layout->setSpacing(0);
        layout->setMargin(0);
    } else {
        layout = new QVBoxLayout;
    }

    assert(initiated_);

    for(std::vector<PluginBase::PluginPtr>::iterator it = meta_plugins_.begin(); it != meta_plugins_.end(); ++it) {

        bool isOk = false;
        for(std::vector<PluginBase::Selector>::const_iterator selector = selectors.begin(); selector != selectors.end(); ++selector) {
            if((*selector)(*it)) {
                isOk = true;
                break;
            }
        }

        //if(!isOk) {
        //    continue;
        //}

        PluginBase::PluginPtr plugin = (*it)->metaInstance();
        result->registerPlugin(plugin);
        result->addMeta(plugin);

        if(isOk) {
            plugin->insert(layout);
        }
    }

    layout->addSpacerItem(new QSpacerItem(0, 1000, QSizePolicy::Minimum, QSizePolicy::Expanding));

    parent->setLayout(layout);
    parent->show();

    return result;
}
