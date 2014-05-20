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

DesignerIO::DesignerIO(Designer &designer, Graph::Ptr graph, WidgetController* widget_ctrl)
    : designer_(designer), graph_(graph), widget_ctrl_(widget_ctrl)
{
}

void DesignerIO::saveSettings(YAML::Emitter& yaml)
{
    yaml << YAML::Key << "view_pos";
    yaml << YAML::Value << YAML::BeginSeq
         << designer_.ui->scrollArea->horizontalScrollBar()->value()
         << designer_.ui->scrollArea->verticalScrollBar()->value()
         << YAML::EndSeq;
}

void DesignerIO::loadSettings(YAML::Node &doc)
{
    QWidget* window = designer_.window();

    if(exists(doc, "view_pos")) {
        int sx, sy;
        doc["view_pos"][0] >> sx;
        doc["view_pos"][1] >> sy;

        designer_.setView(sx, sy);
    }
}

void DesignerIO::saveBoxes(YAML::Emitter& yaml)
{
    boost::function<void(Node*)> cb = boost::bind(&DesignerIO::saveBox, this, _1, boost::ref(yaml));
    graph_->foreachNode(cb);
}

void DesignerIO::saveBox(Node *node, YAML::Emitter &yaml)
{
    Box* box = widget_ctrl_->getBox(node->getUUID());
    NodeAdapter::Ptr na = box->getNodeAdapter();
    Memento::Ptr m = na->getState();
    if(m) {
        m->writeYaml(yaml);
    }
}

void DesignerIO::loadBoxes(YAML::Node &doc)
{
    boost::function<void(Node*)> cb = boost::bind(&DesignerIO::loadBox, this, _1, boost::ref(doc));
    graph_->foreachNode(cb);
}

void DesignerIO::loadBox(Node* node, YAML::Node& doc)
{
    Box* box = widget_ctrl_->getBox(node->getUUID());
    NodeAdapter::Ptr na = box->getNodeAdapter();
    Memento::Ptr m = na->getState();
    if(m) {
        m->readYaml(doc);
        box->getNodeAdapter()->setState(m);
    }
}
