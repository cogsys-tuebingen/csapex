/// HEADER
#include <csapex/graphio.h>

/// PROJECT
#include <csapex/box.h>
#include <csapex/box_manager.h>
#include <csapex/connector_in.h>
#include <csapex/connector_out.h>
#include <csapex/template_manager.h>

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
const std::string GraphIO::config_selector = "Configs(*" + GraphIO::extension + ");;LegacyConfigs(*.vecfg)";

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
    yaml << YAML::Value << graph_->uuids;
    yaml << YAML::Key << "next_template_id";
    yaml << YAML::Value << TemplateManager::instance().next_id;
}

void GraphIO::loadSettings(YAML::Node &doc)
{
    if(doc.FindValue("uuid_map")) {
        doc["uuid_map"] >> graph_->uuids;
    }
    if(doc.FindValue("next_template_id")) {
        doc["next_template_id"] >> TemplateManager::instance().next_id;
    }
}

void GraphIO::saveBoxes(YAML::Emitter &yaml)
{
    BOOST_FOREACH(csapex::Box::Ptr box, graph_->boxes_) {
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

        Box::Ptr box = BoxManager::instance().makeBox(QPoint(x,y), type, uuid);

        if(box) {
            try {
                box->read(doc);

            } catch(const std::exception& e) {
                std::cerr << "cannot load state for box " << uuid << ": " << typeid(e).name() << ", what=" << e.what() << std::endl;
            }

            graph_->addBox(box);

            if(type == "::meta" && !box->state->template_.empty()) {
                SubGraphTemplate::Ptr templ = TemplateManager::instance().get(box->state->template_);
                command::Meta::Ptr meta(new command::Meta);
                templ->createCommands(meta.get(), uuid);

                Command::doExecute(meta);
            }
        }
    }
}

void GraphIO::saveConnections(YAML::Emitter &yaml)
{
    yaml << YAML::Key << "connections";
    yaml << YAML::Value << YAML::BeginSeq; // connections seq

    BOOST_FOREACH(csapex::Box::Ptr box, graph_->boxes_) {
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


    yaml << YAML::Key << "fulcrums";
    yaml << YAML::Value << YAML::BeginSeq; // fulcrums seq

    BOOST_FOREACH(const Connection::Ptr& connection, graph_->visible_connections) {
        if(connection->getFulcrumCount() == 0) {
            continue;
        }

        yaml << YAML::BeginMap;

        yaml << YAML::Key << "from" << YAML::Value << connection->from()->UUID();
        yaml << YAML::Key << "to" << YAML::Value << connection->to()->UUID();

        yaml << YAML::Key << "pts" << YAML::Value << YAML::Flow << YAML::BeginSeq;
        foreach(const QPoint& pt, connection->getFulcrums()) {
            yaml << YAML::Flow << YAML::BeginSeq << pt.x() << pt.y() << YAML::EndSeq;
        }

        yaml << YAML::EndSeq;

        yaml << YAML::EndMap;
    }

    yaml << YAML::EndSeq; // fulcrums seq
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
            if(from_uuid.find(Connector::namespace_separator) == from_uuid.npos) {
                // legacy import
                from_uuid.replace(from_uuid.find("_out_"), 1, Connector::namespace_separator);
            }
            Box::Ptr parent = graph_->findConnectorOwner(from_uuid);

            if(!parent) {
                std::cerr << "cannot find connector '" << from_uuid << "'" << std::endl;
                continue;
            }

            const YAML::Node& targets = connection["targets"];
            assert(targets.Type() == YAML::NodeType::Sequence);

            for(unsigned j=0; j<targets.size(); ++j) {
                std::string to_uuid;
                targets[j] >> to_uuid;

                if(to_uuid.find(Connector::namespace_separator) == to_uuid.npos) {
                    // legacy import
                    to_uuid.replace(to_uuid.find("_in_"), 1, Connector::namespace_separator);
                }

                ConnectorOut* from = parent->getOutput(from_uuid);
                if(from == NULL) {
                    std::cerr << "cannot load connection, connector with uuid '" << from_uuid << "' doesn't exist." << std::endl;
                    continue;
                }

                Box::Ptr target_box = graph_->findConnectorOwner(to_uuid);
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

    if(doc.FindValue("fulcrums")) {
        const YAML::Node& fulcrums = doc["fulcrums"];
        assert(fulcrums.Type() == YAML::NodeType::Sequence);


        for(unsigned i = 0; i < fulcrums.size(); ++i) {
            const YAML::Node& fulcrum = fulcrums[i];
            assert(fulcrum.Type() == YAML::NodeType::Map);

            std::string from_uuid;
            fulcrum["from"] >> from_uuid;
            std::string to_uuid;
            fulcrum["to"] >> to_uuid;

            ConnectorOut* from = dynamic_cast<ConnectorOut*>(graph_->findConnector(from_uuid));
            if(from == NULL) {
                std::cerr << "cannot load fulcrum, connector with uuid '" << from_uuid << "' doesn't exist." << std::endl;
                continue;
            }

            ConnectorIn* to = dynamic_cast<ConnectorIn*>(graph_->findConnector(to_uuid));
            if(to == NULL) {
                std::cerr << "cannot load fulcrum, connector with uuid '" << to_uuid << "' doesn't exist." << std::endl;
                continue;
            }

            Connection::Ptr connection = graph_->getConnection(Connection::Ptr(new Connection(from, to)));

            std::vector< std::vector<int> > pts;
            fulcrum["pts"] >> pts;

            int n = pts.size();
            for(int i = 0; i < n; ++i) {
                connection->addFulcrum(i, QPoint(pts[i][0], pts[i][1]));
            }
        }
    }
}


void GraphIO::saveTemplates(YAML::Emitter &yaml)
{
    TemplateManager& manager = TemplateManager::instance();

    if(manager.temporary_templates.empty()) {
        return;
    }
    yaml << YAML::Key << "temporary_templates";
    yaml << YAML::Value << manager.temporary_templates;
}

void GraphIO::loadTemplates(YAML::Node &doc)
{
    TemplateManager& manager = TemplateManager::instance();
    if(doc.FindValue("temporary_templates")) {
        doc["temporary_templates"] >> manager.temporary_templates;
    }
}
