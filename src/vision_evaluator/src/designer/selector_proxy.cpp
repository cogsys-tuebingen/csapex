/// HEADER
#include "selector_proxy.h"

/// PROJECT
#include <designer/box_manager.h>

using namespace vision_evaluator;

SelectorProxy::SelectorProxy(const std::string& name, QWidget *parent)
    : QGraphicsView(parent), name_(name)
{
}

SelectorProxy::~SelectorProxy()
{
}

void SelectorProxy::registerProxy(ProxyConstructor c)
{
    BoxManager::instance().register_box_type(c);\
}
