/// HEADER
#include <csapex/view/designer/designerio.h>

/// PROJECT
#include <csapex/view/designer/designer.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_handle.h>
#include <csapex/view/node/box.h>
#include <csapex/view/node/node_adapter.h>
#include <csapex/utility/assert.h>
#include <csapex/utility/yaml_io.hpp>
#include <csapex/view/designer/graph_view.h>
#include <csapex/model/graph_facade.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph/vertex.h>


/// SYSTEM
#include <QMessageBox>
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <QScrollBar>
#include <sys/types.h>

using namespace csapex;

DesignerIO::DesignerIO()
{
}

void DesignerIO::saveSettings(YAML::Node& /*yaml*/)
{
}

void DesignerIO::loadSettings(YAML::Node &/*doc*/)
{
}

void DesignerIO::saveBoxes(YAML::Node& yaml, Graph* graph, GraphView *view)
{
    YAML::Node adapters(YAML::NodeType::Sequence);
    for(auto it = graph->beginVertices(); it != graph->endVertices(); ++it) {
        NodeHandlePtr nh = (*it)->getNodeHandle();
        saveBox(nh.get(), view, adapters);
    }
    yaml["adapters"] = adapters;
}

void DesignerIO::saveBox(NodeHandle *node, GraphView *view, YAML::Node &yaml)
{
    NodeBox* box = view->getBox(node->getUUID());
    NodeAdapter::Ptr na = box->getNodeAdapter();
    Memento::Ptr m = na->getState();
    if(m) {
        YAML::Node doc;
        doc["uuid"] = node->getUUID().getFullName();

        YAML::Node state;
        m->writeYaml(state);
        doc["state"] = state;

        yaml.push_back(doc);
    }
}

void DesignerIO::loadBoxes(YAML::Node &doc, GraphView *view)
{
    std::shared_ptr<UUIDProvider> graph = view->getGraphFacade()->getGraph()->shared_from_this();

    if(doc["adapters"].IsDefined()) {
        const YAML::Node& adapters = doc["adapters"];
        for(std::size_t i = 0; i < adapters.size(); ++i) {
            const YAML::Node& e = adapters[i];

            YAML::Node x = e["uuid"];
            apex_assert_hard(x.Type() == YAML::NodeType::Scalar);

            UUID uuid = UUIDProvider::makeUUID_forced(graph, e["uuid"].as<std::string>());

            NodeBox* box = view->getBox(uuid);
            if(box) {
                NodeAdapter::Ptr na = box->getNodeAdapter();
                Memento::Ptr m = na->getState();
                if(m) {
                    m->readYaml(e["state"]);
                    box->getNodeAdapter()->setParameterState(m);
                }
            }
        }
    }
}
