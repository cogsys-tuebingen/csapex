/// HEADER
#include <csapex/view/designer/designerio.h>

/// PROJECT
#include <csapex/view/designer/designer.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>
#include <csapex/view/designer/widget_controller.h>
#include <csapex/view/node/box.h>
#include <csapex/view/node/node_adapter.h>
#include <csapex/utility/assert.h>
#include <csapex/utility/yaml_io.hpp>

/// SYSTEM
#include <QMessageBox>

#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <QScrollBar>
#include <sys/types.h>
#include <pwd.h>

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

void DesignerIO::saveBoxes(YAML::Node& yaml, Graph* graph, WidgetController* widget_ctrl)
{
    for(auto it = graph->beginNodes(); it != graph->endNodes(); ++it) {
        NodeWorkerPtr nh = *it;
        YAML::Node adapters;
        saveBox(nh.get(), widget_ctrl, adapters);
        yaml["adapters"] = adapters;
    }
}

void DesignerIO::saveBox(NodeHandle *node, WidgetController* widget_ctrl, YAML::Node &yaml)
{
    NodeBox* box = widget_ctrl->getBox(node->getUUID());
    NodeAdapter::Ptr na = box->getNodeAdapter();
    Memento::Ptr m = na->getState();
    if(m) {
        YAML::Node doc;
        doc["uuid"] = node->getUUID();

        YAML::Node state;
        m->writeYaml(state);
        doc["state"] = state;

        yaml.push_back(doc);
    }
}

void DesignerIO::loadBoxes(YAML::Node &doc, WidgetController* widget_ctrl)
{
    if(doc["adapters"].IsDefined()) {
        const YAML::Node& adapters = doc["adapters"];
        for(std::size_t i = 0; i < adapters.size(); ++i) {
            const YAML::Node& e = adapters[i];

            UUID uuid = e["uuid"].as<UUID>();

            NodeBox* box = widget_ctrl->getBox(uuid);
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
