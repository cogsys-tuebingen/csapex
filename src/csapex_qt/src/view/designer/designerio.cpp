/// HEADER
#include <csapex/view/designer/designerio.h>

/// PROJECT
#include <csapex/model/graph_facade.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph/vertex.h>
#include <csapex/model/generic_state.h>
#include <csapex/utility/assert.h>
#include <csapex/utility/yaml_io.hpp>
#include <csapex/view/designer/designer.h>
#include <csapex/view/designer/graph_view.h>
#include <csapex/view/node/box.h>
#include <csapex/view/node/node_adapter.h>

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

void DesignerIO::saveBoxes(YAML::Node& yaml, const GraphFacade* graph, GraphView* view)
{
    YAML::Node adapters(YAML::NodeType::Sequence);
    for (const UUID& uuid : graph->enumerateAllNodes()) {
        saveBox(uuid, view, adapters);
    }
    yaml["adapters"] = adapters;
}

void DesignerIO::saveBox(const UUID& node_uuid, GraphView* view, YAML::Node& yaml)
{
    NodeBox* box = view->getBox(node_uuid);
    NodeAdapter::Ptr na = box->getNodeAdapter();
    GenericStatePtr m = na->getState();
    if (m) {
        YAML::Node doc;
        doc["uuid"] = node_uuid.getFullName();

        YAML::Node state;
        m->writeYaml(state);
        doc["state"] = state;

        yaml.push_back(doc);
    }
}

void DesignerIO::loadBoxes(const YAML::Node& doc, GraphView* view)
{
    if (doc["adapters"].IsDefined()) {
        const YAML::Node& adapters = doc["adapters"];
        for (std::size_t i = 0; i < adapters.size(); ++i) {
            const YAML::Node& e = adapters[i];

            YAML::Node x = e["uuid"];
            apex_assert_hard(x.Type() == YAML::NodeType::Scalar);

            UUID uuid = UUIDProvider::makeUUID_without_parent(e["uuid"].as<std::string>());

            NodeBox* box = view->getBox(uuid);
            if (box) {
                NodeAdapter::Ptr na = box->getNodeAdapter();
                na->readLegacyYaml(e["state"]);

                /// deprecated:
                GenericStatePtr m = na->getState();
                if (m) {
                    m->readYaml(e["state"]);
                    box->getNodeAdapter()->setParameterState(m);
                }
            }
        }
    }
}
