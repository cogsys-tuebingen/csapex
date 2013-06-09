#ifndef BOX_MANAGER_H
#define BOX_MANAGER_H

/// COMPONENT
#include "selector_proxy.h"

/// PROJECT
#include <evaluator/generic_manager.hpp>

/// SYSTEM
#include <QLayout>
#include <vector>

namespace vision_evaluator {

class BoxManager
{
public:
    static BoxManager& instance() {
        static BoxManager inst;
        return inst;
    }

public:
    void register_box_type(SelectorProxy::ProxyConstructor provider);
    void fill(QLayout* layout);

protected:
    BoxManager();
    BoxManager(const BoxManager& copy);
    BoxManager& operator = (const BoxManager& assign);

    PluginManager<SelectorProxy, SelectorProxy::ProxyConstructor> available_elements;
};

}

#endif // BOX_MANAGER_H
