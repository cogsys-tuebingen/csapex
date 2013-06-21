/// HEADER
#include "designerio.h"

/// PROJECT
#include "designer.h"
#include "box.h"
#include "box_manager.h"
#include "ui_designer.h"

/// SYSTEM
#include <boost/foreach.hpp>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

using namespace vision_evaluator;

DesignerIO::DesignerIO()
{
}

void DesignerIO::save(Designer* designer)
{
    YAML::Emitter yaml;

    // settings
    yaml << YAML::BeginMap;

    QSize window_size = designer->window()->size();
    yaml << YAML::Key << "window_size";
    yaml << YAML::Value << YAML::BeginSeq << window_size.width() << window_size.height() << YAML::EndSeq;

    QPoint window_pos = designer->window()->pos();
    yaml << YAML::Key << "window_pos";
    yaml << YAML::Value << YAML::BeginSeq << window_pos.x() << window_pos.y() << YAML::EndSeq;

    yaml << YAML::EndMap;

    QList<vision_evaluator::Box*> boxes = designer->findChildren<vision_evaluator::Box*> ();
    BOOST_FOREACH(vision_evaluator::Box* box, boxes) {
        yaml << *box;
    }

    std::ofstream ofs("ve_config.yaml");
    ofs << yaml.c_str();

    std::cout << "save: " << yaml.c_str() << std::endl;

    BoxManager::instance().setDirty(false);
}

void DesignerIO::load(Designer* designer)
{
    designer->clear();

    std::map<std::string, Box*> loaded_boxes;

    loadBoxes(designer, loaded_boxes);
    loadConnections(designer, loaded_boxes);
}

void DesignerIO::loadBoxes(Designer* designer, std::map<std::string, Box*>& loaded_boxes)
{
    std::ifstream ifs("ve_config.yaml");
    YAML::Parser parser(ifs);

    YAML::Node doc;

    if(!parser.GetNextDocument(doc)) {
        std::cerr << "cannot read the config" << std::endl;
        return;
    }

    QWidget* window = designer->window();//QApplication::activeWindow();

    int w = window->width();
    int h = window->height();
    int x = window->pos().x();
    int y = window->pos().y();

    if(doc.FindValue("window_size")) {
        doc["window_size"][0] >> w;
        doc["window_size"][1] >> h;
    }

    if(doc.FindValue("window_pos")) {
        doc["window_pos"][0] >> x;
        doc["window_pos"][1] >> y;
    }

    window->setGeometry(x,y,w,h);

    while(parser.GetNextDocument(doc)) {
        std::string type, uuid;
        doc["type"] >> type;
        doc["uuid"] >> uuid;

        int x, y;
        doc["pos"][0] >> x;
        doc["pos"][1] >> y;

        std::cout << "create box: " << type << " @ " << x << ", " << y << std::endl;

        Box* box = BoxManager::instance().makeBox(designer->ui->designer, QPoint(x,y), type, uuid);
        loaded_boxes[uuid] = box;

        if(doc.FindValue("state")) {
            const YAML::Node& state_map = doc["state"];
            Memento::Ptr state = box->getState();
            state->readYaml(state_map);
            box->setState(state);
        }
    }
}
void DesignerIO::loadConnections(Designer* designer, std::map<std::string, Box*>& loaded_boxes)
{
    std::ifstream ifs("ve_config.yaml");
    YAML::Parser parser(ifs);

    YAML::Node doc;
    if(!parser.GetNextDocument(doc)) {
        /// SKIP THIS CONFIG PART
        std::cerr << "cannot read the config" << std::endl;
        return;
    }

    while(parser.GetNextDocument(doc)) {
        std::string uuid;
        doc["uuid"] >> uuid;

        if(doc.FindValue("connections")) {
            const YAML::Node& connections = doc["connections"];
            if(connections.FindValue("out")) {
                const YAML::Node& out_list = connections["out"];
                assert(out_list.Type() == YAML::NodeType::Sequence);

                for(unsigned i=0; i<out_list.size(); ++i) {
                    const YAML::Node& connector = out_list[i];
                    assert(connector.Type() == YAML::NodeType::Map);
                    std::string from_uuid;
                    connector["uuid"] >> from_uuid;

                    const YAML::Node& targets = connector["targets"];
                    assert(targets.Type() == YAML::NodeType::Sequence);

                    for(unsigned j=0; j<targets.size(); ++j) {
                        std::string to_uuid;
                        targets[j] >> to_uuid;

                        Box* from_box = loaded_boxes[uuid];
                        if(from_box == NULL) {
                            std::cerr << "cannot load connection, connector with uuid '" << from_uuid << "' doesn't exist." << std::endl;
                            continue;
                        }
                        ConnectorOut* from = from_box->getOutput(from_uuid);

                        ConnectorIn* to = NULL;

                        typedef std::pair<std::string, Box*> Pair;
                        BOOST_FOREACH(Pair p, loaded_boxes) {
                            Box* b = p.second;
                            to = b->getInput(to_uuid);
                            if(to != NULL) {
                                break;
                            }
                        }

                        if(to == NULL) {
                            std::cerr << "cannot load connection, connector with uuid '" << to_uuid << "' doesn't exist." << std::endl;
                            continue;
                        }
                        if(from->tryConnect(to)) {
                            designer->ui->designer->getOverlay()->addConnection(from, to);
                        }
                    }
                }

            }
        }
    }
}
