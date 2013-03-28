/// HEADER
#include "option.h"

/// SYSTEM
#include <QHBoxLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QSpacerItem>

REGISTER_META_PLUGIN(Option)

using namespace vision_evaluator;

Option::Option(const std::string& label)
    : Plugin(label)
{
}

Option::~Option()
{
}

PluginBase::PluginPtr Option::createMetaInstance()
{
    return PluginPtr(new Option("Options"));
}

PluginBase::PluginPtr Option::metaInstance()
{
    return Option::createMetaInstance();
}

void Option::refresh()
{
    active_options.clear();
    std::vector<Option::Constructor>& constructors = instance().constructors;
    for(std::vector<Option::Constructor>::iterator it = constructors.begin(); it != constructors.end(); ++it) {
        Option::TypePtr option = (*it)(CONSTRUCTOR_META);
        active_options.push_back(option);
    }
}

void Option::insert(QBoxLayout* layout)
{
    displayed_options.clear();

    std::vector<Option::Constructor>&constructors = instance().constructors;
    for(std::vector<Option::Constructor>::iterator it = constructors.begin(); it != constructors.end(); ++it) {
        QGroupBox* group = new QGroupBox;
        layout->addWidget(group);

        Option::TypePtr option = (*it)(CONSTRUCTOR_META);
        QObject::connect(option.get(), SIGNAL(plugin_changed()), this, SIGNAL(plugin_changed()));
        //option->plugin_changed.connect(plugin_changed);

        QBoxLayout* child_layout = new QVBoxLayout;
        group->setLayout(child_layout);
        group->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding);
        group->setTitle(option->getName().c_str());

        option->insert(child_layout);
        displayed_options.push_back(option);
    }
}
