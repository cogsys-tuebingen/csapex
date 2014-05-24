/// HEADER
#include <csapex/core/designerio.h>

/// PROJECT
#include <csapex/view/designer.h>
#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/view/widget_controller.h>
#include <csapex/view/box.h>
#include <csapex/view/node_adapter.h>
#include "ui_designer.h"

/// SYSTEM
#include <QMessageBox>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <utils_yaml/yamlplus.h>
#include <QScrollBar>
#include <sys/types.h>
#include <pwd.h>

using namespace csapex;

DesignerIO::DesignerIO(Designer &designer)
    : designer_(designer)
{
}

void DesignerIO::saveSettings(YAML::Emitter& yaml)
{
    yaml << YAML::Key << "view_pos";
    yaml << YAML::Value << YAML::BeginSeq
//         << designer_.ui->scrollArea->horizontalScrollBar()->value()
//         << designer_.ui->scrollArea->verticalScrollBar()->value()
         << YAML::EndSeq;
}

void DesignerIO::loadSettings(YAML::Node &/*doc*/)
{
//    QWidget* window = designer_.window();

//    if(exists(doc, "view_pos")) {
//        int sx, sy;
//        doc["view_pos"][0] >> sx;
//        doc["view_pos"][1] >> sy;

//        designer_.setView(sx, sy);
//    }
}

void DesignerIO::saveBoxes(YAML::Emitter& yaml, Graph::Ptr graph, WidgetController* widget_ctrl)
{
    yaml << YAML::Key << "adapters";
    yaml << YAML::Value << YAML::BeginSeq; // adapters seq


    boost::function<void(Node*)> cb = boost::bind(&DesignerIO::saveBox, this, _1, widget_ctrl, boost::ref(yaml));
    graph->foreachNode(cb);

    yaml << YAML::EndSeq; // adapters seq
}

void DesignerIO::saveBox(Node *node, WidgetController* widget_ctrl, YAML::Emitter &yaml)
{
    NodeBox* box = widget_ctrl->getBox(node->getUUID());
    NodeAdapter::Ptr na = box->getNodeAdapter();
    Memento::Ptr m = na->getState();
    if(m) {
        yaml << YAML::BeginMap;
        yaml << YAML::Key << "uuid" << YAML::Value << na->getNode()->getUUID();
        yaml << YAML::Key << "state" << YAML::Value;

        yaml << YAML::BeginMap;
        m->writeYaml(yaml);
        yaml << YAML::EndMap;

        yaml << YAML::EndMap;
    }
}

void DesignerIO::loadBoxes(YAML::Node &doc, WidgetController* widget_ctrl)
{
    if(exists(doc, "adapters")) {
        const YAML::Node& adapters = doc["adapters"];
        assert(adapters.Type() == YAML::NodeType::Sequence);

        for(std::size_t i = 0; i < adapters.size(); ++i) {
            const YAML::Node& e = adapters[i];

            std::string uuid;
            e["uuid"] >> uuid;

            NodeBox* box = widget_ctrl->getBox(UUID::make_forced(uuid));
            if(box) {
                NodeAdapter::Ptr na = box->getNodeAdapter();
                Memento::Ptr m = na->getState();
                if(m) {
                    m->readYaml(e["state"]);
                    box->getNodeAdapter()->setState(m);
                }
            }
        }
    }
}
