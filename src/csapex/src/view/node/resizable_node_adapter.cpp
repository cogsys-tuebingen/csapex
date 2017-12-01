/// HEADER
#include <csapex/view/node/resizable_node_adapter.h>

/// PROJECT
#include <csapex/model/node_facade.h>
#include <csapex/model/node_state.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

using namespace csapex;

ResizableNodeAdapter::ResizableNodeAdapter(NodeFacadePtr worker, NodeBox* parent)
    : DefaultNodeAdapter(worker, parent),
      initialized_(false)
{
    size_.setWidth(100);
    size_.setHeight(100);
}

bool ResizableNodeAdapter::isResizable() const
{
    return true;
}

void ResizableNodeAdapter::readLegacyYaml(const YAML::Node& node)
{
    if(node["width"].IsDefined()) {
        size_.setWidth(node["width"].as<int>());
    }
    if(node["height"].IsDefined()) {
        size_.setHeight(node["height"].as<int>());
    }

    setSize(size_.width(), size_.height());

    readState();

    doResize();
}

void ResizableNodeAdapter::doResize()
{
    resize(size_);
}


void ResizableNodeAdapter::setSize(int width, int height)
{
    if(!initialized_) {
        return;
    }

    size_.setWidth(width);
    size_.setHeight(height);

    if(NodeFacadePtr node_handle = node_.lock()) {
        NodeStatePtr node_state = node_handle->getNodeState();
        node_state->setDictionaryEntry("width", size_.width());
        node_state->setDictionaryEntry("height", size_.height());
    }
}

void ResizableNodeAdapter::readState()
{
    if(NodeFacadePtr node_handle = node_.lock()) {
        NodeStatePtr node_state = node_handle->getNodeState();
        size_.setWidth(node_state->getDictionaryEntry("width", size_.width()));
        size_.setHeight(node_state->getDictionaryEntry("height", size_.height()));
    }
}

void ResizableNodeAdapter::setupUi(QBoxLayout* layout)
{
    DefaultNodeAdapter::setupUi(layout);

    readState();
    resize(size_);

    initialized_ = true;
}
