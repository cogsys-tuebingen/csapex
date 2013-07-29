/// HEADER
#include "designerio.h"

/// PROJECT
#include "designer.h"
#include "box.h"
#include "box_manager.h"
#include "ui_designer.h"

/// SYSTEM
#include <QMessageBox>
#include <boost/foreach.hpp>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <QScrollBar>

using namespace vision_evaluator;

const std::string DesignerIO::extension = ".vecfg";
const std::string DesignerIO::default_config = "default" + DesignerIO::extension;
const std::string DesignerIO::config_selector = "Configs (*" + DesignerIO::extension + ")";

DesignerIO::DesignerIO()
{
}

void DesignerIO::save(Designer* designer, const std::string& file)
{
    if(file.empty()) {
        std::cerr << "cannot save, no file specified" << std::endl;
        return;
    }

    YAML::Emitter yaml;

    // settings
    yaml << YAML::BeginMap; // settings map

    QSize window_size = designer->window()->size();
    yaml << YAML::Key << "window_size";
    yaml << YAML::Value << YAML::BeginSeq << window_size.width() << window_size.height() << YAML::EndSeq;

    QPoint window_pos = designer->window()->pos();
    yaml << YAML::Key << "window_pos";
    yaml << YAML::Value << YAML::BeginSeq << window_pos.x() << window_pos.y() << YAML::EndSeq;

    yaml << YAML::Key << "uuid_map";
    yaml << YAML::Value << BoxManager::instance().uuids;


    QList<vision_evaluator::Box*> boxes = designer->findChildren<vision_evaluator::Box*> ();

    // connections
    saveConnections(yaml, boxes); // connections map

    yaml << YAML::EndMap; // settings map

    // boxes
    BOOST_FOREACH(vision_evaluator::Box* box, boxes) {
        yaml << *box;
    }

    std::ofstream ofs(file.c_str());
    ofs << yaml.c_str();

    std::cout << "save: " << yaml.c_str() << std::endl;

    BoxManager::instance().setDirty(false);
}

void DesignerIO::load(Designer* designer, const std::string& file)
{
    if(file.empty()) {
        std::cerr << "cannot load, no file specified" << std::endl;
        return;
    }

    QFile test(file.c_str());
    if(!test.exists()) {
        std::cerr << "cannot load " << file << ", doesn't exist" << std::endl;
        return;
    }

    designer->clear();

    loadBoxes(designer, file);
    loadConnections(designer, file);

    BoxManager::instance().setDirty(false);
}

void DesignerIO::loadSettings(Designer* designer, YAML::Node &doc)
{
    QWidget* window = designer->window();

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

    if(doc.FindValue("uuid_map")) {
        doc["uuid_map"] >> BoxManager::instance().uuids;
    }
}

void DesignerIO::loadBoxes(Designer* designer, const std::string &file)
{
    std::ifstream ifs(file.c_str());
    YAML::Parser parser(ifs);

    YAML::Node doc;

    if(!parser.GetNextDocument(doc)) {
        std::cerr << "cannot read the config" << std::endl;
        return;
    }

    loadSettings(designer, doc);

    while(parser.GetNextDocument(doc)) {
        std::string type, uuid;
        doc["type"] >> type;
        doc["uuid"] >> uuid;

        int x, y;
        doc["pos"][0] >> x;
        doc["pos"][1] >> y;

        std::cout << "create box: " << type << " @ " << x << ", " << y << std::endl;

        Box* box = BoxManager::instance().makeBox(designer->ui->designer, QPoint(x,y), type, uuid);

        if(box) {
            Memento::Ptr s = box->getState();
            s->readYaml(doc);
            box->setState(s);
        }
    }
}

void DesignerIO::saveConnections(YAML::Emitter &yaml, QList<vision_evaluator::Box*>& boxes)
{
    yaml << YAML::Key << "connections";
    yaml << YAML::Value << YAML::BeginSeq; // connections seq

    BOOST_FOREACH(vision_evaluator::Box* box, boxes) {
        if(!box->output.empty()) {
            BOOST_FOREACH(ConnectorOut* o, box->output) {
                if(o->beginTargets() == o->endTargets()) {
                    continue;
                }
                yaml << YAML::BeginMap; // output map
                yaml << YAML::Key << "uuid";
                yaml << YAML::Value << o->UUID();
                yaml << YAML::Key << "targets";
                yaml << YAML::Value << YAML::BeginSeq; // output list
                for(ConnectorOut::TargetIterator it = o->beginTargets(); it != o->endTargets(); ++it) {
                    ConnectorIn* i = *it;
                    yaml << i->UUID();
                }
                yaml << YAML::EndSeq; // output list

                yaml << YAML::EndMap; // output map
            }
        }
    }

    yaml << YAML::EndSeq; // connections seq
}

void DesignerIO::loadConnections(Designer* designer, const std::string &file)
{
    std::ifstream ifs(file.c_str());
    YAML::Parser parser(ifs);

    YAML::Node doc;
    if(!parser.GetNextDocument(doc)) {
        /// SKIP THIS CONFIG PART
        std::cerr << "cannot read the config" << std::endl;
        return;
    }

    if(doc.FindValue("connections")) {
        const YAML::Node& connections = doc["connections"];
        assert(connections.Type() == YAML::NodeType::Sequence);

        for(unsigned i = 0; i < connections.size(); ++i) {
            const YAML::Node& connection = connections[i];
            assert(connection.Type() == YAML::NodeType::Map);

            std::string from_uuid;
            connection["uuid"] >> from_uuid;
            Box* parent = BoxManager::instance().findConnectorOwner(from_uuid);

            if(!parent) {
                std::cerr << "cannot find connector '" << from_uuid << "'" << std::endl;
                continue;
            }

            const YAML::Node& targets = connection["targets"];
            assert(targets.Type() == YAML::NodeType::Sequence);

            for(unsigned j=0; j<targets.size(); ++j) {
                std::string to_uuid;
                targets[j] >> to_uuid;

                ConnectorOut* from = parent->getOutput(from_uuid);
                if(from == NULL) {
                    std::cerr << "cannot load connection, connector with uuid '" << from_uuid << "' doesn't exist." << std::endl;
                    continue;
                }

                Box* target_box = BoxManager::instance().findConnectorOwner(to_uuid);
                if(target_box == NULL) {
                    std::cerr << "cannot load connection, connector with uuid '" << to_uuid << "' doesn't exist." << std::endl;
                    continue;
                }

                ConnectorIn* to = target_box->getInput(to_uuid);
                assert(to); // if parent box has been found, this should never happen

                std::cout << "connection: " << from_uuid << " -> " << to_uuid << std::endl;
                from->connectForcedWithoutCommand(to);
            }
        }
    }
}
