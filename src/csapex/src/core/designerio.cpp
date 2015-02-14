/// HEADER
#include <csapex/core/designerio.h>

/// PROJECT
#include <csapex/view/designer.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_worker.h>
#include <csapex/view/widget_controller.h>
#include <csapex/view/box.h>
#include <csapex/view/node_adapter.h>
#include "ui_designer.h"
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
    std::function<void(NodeWorker*)> cb = std::bind(&DesignerIO::saveBox, this, std::placeholders::_1, widget_ctrl, yaml["adapters"]);
    graph->foreachNode(cb);
}

void DesignerIO::saveBox(NodeWorker *node, WidgetController* widget_ctrl, YAML::Node &yaml)
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
