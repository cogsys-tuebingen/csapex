/// HEADER
#include <csapex/graphio.h>

/// PROJECT
#include <csapex/box.h>
#include <csapex/box_manager.h>
#include <csapex/connector_in.h>
#include <csapex/connector_out.h>

/// SYSTEM
#include <QMessageBox>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <QScrollBar>
#include <sys/types.h>
#include <pwd.h>

using namespace csapex;

const std::string GraphIO::extension = ".apex";
const std::string GraphIO::default_config = GraphIO::defaultConfigFile();
const std::string GraphIO::config_selector = "Configs (*" + GraphIO::extension + "), LegacyConfigs(*.vecfg)";

std::string GraphIO::defaultConfigPath()
{
    struct passwd *pw = getpwuid(getuid());
    return std::string(pw->pw_dir) + "/.csapex/";
}

std::string GraphIO::defaultConfigFile()
{
    std::string dir = GraphIO::defaultConfigPath();

    if(!boost::filesystem3::exists(dir)) {
        boost::filesystem3::create_directories(dir);
    }

    std::string file = dir + "default" + GraphIO::extension;

    if(!boost::filesystem3::exists(file)) {
//        createDefaultConfig(file);
    }

    return file;
}

GraphIO::GraphIO(Graph::Ptr graph)
    : graph_(graph)
{
}


void GraphIO::saveSettings(YAML::Emitter& yaml)
{
    yaml << YAML::Key << "uuid_map";
    yaml << YAML::Value << BoxManager::instance().uuids;
}

void GraphIO::loadSettings(YAML::Node &doc)
{
    if(doc.FindValue("uuid_map")) {
        doc["uuid_map"] >> BoxManager::instance().uuids;
    }
}

void GraphIO::saveBoxes(YAML::Emitter &yaml)
{
    BOOST_FOREACH(csapex::Box* box, graph_->boxes_) {
        box->save(yaml);
    }
}

void GraphIO::loadBoxes(YAML::Parser& parser)
{
    YAML::Node doc;

    while(parser.GetNextDocument(doc)) {
        std::string type, uuid;
        doc["type"] >> type;
        doc["uuid"] >> uuid;

        int x, y;
        doc["pos"][0] >> x;
        doc["pos"][1] >> y;

        Box* box = BoxManager::instance().makeBox(QPoint(x,y), type, uuid);

        if(box) {
            Memento::Ptr s = box->getState();
            s->readYaml(doc);
            box->setState(s);

            graph_->addBox(box);
        }
    }
}

void GraphIO::saveConnections(YAML::Emitter &yaml)
{
    yaml << YAML::Key << "connections";
    yaml << YAML::Value << YAML::BeginSeq; // connections seq

    BOOST_FOREACH(csapex::Box* box, graph_->boxes_) {
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

void GraphIO::loadConnections(YAML::Node &doc)
{
    if(doc.FindValue("connections")) {
        const YAML::Node& connections = doc["connections"];
        assert(connections.Type() == YAML::NodeType::Sequence);

        for(unsigned i = 0; i < connections.size(); ++i) {
            const YAML::Node& connection = connections[i];
            assert(connection.Type() == YAML::NodeType::Map);

            std::string from_uuid;
            connection["uuid"] >> from_uuid;
            Box* parent = graph_->findConnectorOwner(from_uuid);

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

                Box* target_box = graph_->findConnectorOwner(to_uuid);
                if(target_box == NULL) {
                    std::cerr << "cannot load connection, connector with uuid '" << to_uuid << "' doesn't exist." << std::endl;
                    continue;
                }

                ConnectorIn* to = target_box->getInput(to_uuid);
                assert(to); // if parent box has been found, this should never happen

                graph_->addConnection(Connection::Ptr(new Connection(from, to)));
            }
        }
    }
}

