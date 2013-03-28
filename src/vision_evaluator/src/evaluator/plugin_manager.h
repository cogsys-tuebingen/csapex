#ifndef PLUGINMANAGER_H
#define PLUGINMANAGER_H

/// COMPONENT
#include "plugin_base.h"
#include "plugin_queue.h"

/// SYSTEM
#include <opencv2/opencv.hpp>
#include <boost/function.hpp>
#include <map>
#include <QWidget>

namespace vision_evaluator
{

class PluginManager
{
private:
    PluginManager();

public:
    ~PluginManager();

    static PluginManager& instance();

    typedef boost::function<void(int argc, char** argv)> MetaInit;
    typedef boost::function<PluginBase::PluginPtr()> MetaConstructor;

public:
    PluginQueue::Ptr createQueue(QWidget* parent, QToolBox* toolbox, const std::vector<PluginBase::Selector>& selectors
                                 = std::vector<PluginBase::Selector>(), bool horizontal = false);
    void register_meta_plugin(MetaInit init, MetaConstructor meta);

    void init(int argc, char** argv);

private:
    std::vector<std::pair<MetaInit, MetaConstructor> > meta_plugin_constructors_;
    std::vector<PluginBase::PluginPtr> meta_plugins_;

    std::map<QWidget*, std::vector<PluginBase::PluginPtr> > instances;

    bool initiated_;
};

} /// NAMESPACE

#endif // PLUGINMANAGER_H
