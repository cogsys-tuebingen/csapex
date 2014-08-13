/// HEADER
#include <csapex/core/graphio.h>

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_factory.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/fulcrum.h>
#include <csapex/utility/assert.h>
#include <csapex/model/node_state.h>

/// SYSTEM
#include <QMessageBox>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <QScrollBar>
#include <sys/types.h>

using namespace csapex;

GraphIO::GraphIO(Graph *graph, NodeFactory* node_factory)
    : graph_(graph), node_factory_(node_factory)
{
}


void GraphIO::saveSettings(YAML::Emitter& yaml)
{
    yaml << YAML::Key << "uuid_map";
    yaml << YAML::Value << graph_->uuids_;
}

void GraphIO::loadSettings(YAML::Node &doc)
{
    if(exists(doc, "uuid_map")) {
        doc["uuid_map"] >> graph_->uuids_;
    }
}

void GraphIO::saveNodes(YAML::Emitter &yaml)
{
    BOOST_FOREACH(Node::Ptr node, graph_->nodes_) {
        node->getNodeState()->writeYaml(yaml);
    }
}

void GraphIO::loadNodes(YAML::Parser& parser)
{
    YAML::Node doc;

    while(getNextDocument(parser, doc)) {
        std::string type, uuid_tmp;
        doc["type"] >> type;
        doc["uuid"] >> uuid_tmp;

        UUID uuid = UUID::make_forced(uuid_tmp);

        int x, y;
        doc["pos"][0] >> x;
        doc["pos"][1] >> y;

        Node::Ptr node = node_factory_->makeNode(type, uuid);
        if(!node) {
            continue;
        }
        try {
            NodeState::Ptr s = node->getNodeState();
            s->readYaml(doc);
            node->setNodeState(s);

        } catch(const std::exception& e) {
            std::cerr << "cannot load state for box " << uuid << ": " << typeid(e).name() << ", what=" << e.what() << std::endl;
        }
        if(x != 0 || y != 0) {
            node->getNodeState()->setPos(QPoint(x,y));
        }
        graph_->addNode(node);
    }
}

void GraphIO::saveConnections(YAML::Emitter &yaml)
{
    yaml << YAML::Key << "connections";
    yaml << YAML::Value << YAML::BeginSeq; // connections seq

    BOOST_FOREACH(Node::Ptr node, graph_->nodes_) {
        if(!node->getAllOutputs().empty()) {
            BOOST_FOREACH(Output* o, node->getAllOutputs()) {
                if(o->beginTargets() == o->endTargets()) {
                    continue;
                }
                yaml << YAML::BeginMap; // output map
                yaml << YAML::Key << "uuid";
                yaml << YAML::Value << o->getUUID();
                yaml << YAML::Key << "targets";
                yaml << YAML::Value << YAML::BeginSeq; // output list
                for(Output::TargetIterator it = o->beginTargets(); it != o->endTargets(); ++it) {
                    Input* i = *it;
                    yaml << i->getUUID();
                }
                yaml << YAML::EndSeq; // output list

                yaml << YAML::EndMap; // output map
            }
        }
    }

    yaml << YAML::EndSeq; // connections seq


    yaml << YAML::Key << "fulcrums";
    yaml << YAML::Value << YAML::BeginSeq; // fulcrums seq

    BOOST_FOREACH(const Connection::Ptr& connection, graph_->connections_) {
        if(connection->getFulcrumCount() == 0) {
            continue;
        }

        yaml << YAML::BeginMap;

        yaml << YAML::Key << "from" << YAML::Value << connection->from()->getUUID();
        yaml << YAML::Key << "to" << YAML::Value << connection->to()->getUUID();

        yaml << YAML::Key << "pts" << YAML::Value << YAML::Flow << YAML::BeginSeq;
        Q_FOREACH(const Fulcrum::Ptr& f, connection->getFulcrums()) {
            yaml << YAML::Flow << YAML::BeginSeq << f->pos().x() << f->pos().y() << YAML::EndSeq;
        }
        yaml << YAML::EndSeq;

        yaml << YAML::Key << "handles" << YAML::Value << YAML::Flow << YAML::BeginSeq;
        Q_FOREACH(const Fulcrum::Ptr& f, connection->getFulcrums()) {
            yaml << YAML::Flow << YAML::BeginSeq << f->handleIn().x() << f->handleIn().y() << f->handleOut().x() << f->handleOut().y() << YAML::EndSeq;
        }
        yaml << YAML::EndSeq;

        yaml << YAML::Key << "types" << YAML::Value << YAML::Flow << YAML::BeginSeq;
        Q_FOREACH(const Fulcrum::Ptr& f, connection->getFulcrums()) {
            yaml << f->type();
        }
        yaml << YAML::EndSeq;

        yaml << YAML::EndMap;
    }

    yaml << YAML::EndSeq; // fulcrums seq
}

void GraphIO::loadConnections(YAML::Node &doc)
{
    if(exists(doc, "connections")) {
        const YAML::Node& connections = doc["connections"];
        apex_assert_hard(connections.Type() == YAML::NodeType::Sequence);

        for(unsigned i = 0; i < connections.size(); ++i) {
            const YAML::Node& connection = connections[i];
            apex_assert_hard(connection.Type() == YAML::NodeType::Map);

            std::string from_uuid_tmp;
            connection["uuid"] >> from_uuid_tmp;
            if(from_uuid_tmp.find(UUID::namespace_separator) == from_uuid_tmp.npos) {
                // legacy import
                from_uuid_tmp.replace(from_uuid_tmp.find("_out_"), 1, UUID::namespace_separator);
            }

            UUID from_uuid = UUID::make_forced(from_uuid_tmp);

            Node* parent = NULL;
            try {
                parent = graph_->findNodeForConnector(from_uuid);

            } catch(const std::exception& e) {
                std::cerr << "cannot find connector '" << from_uuid << "'" << std::endl;
                continue;
            }

            if(!parent) {
                std::cerr << "cannot find connector '" << from_uuid << "'" << std::endl;
                continue;
            }

            const YAML::Node& targets = connection["targets"];
            apex_assert_hard(targets.Type() == YAML::NodeType::Sequence);

            for(unsigned j=0; j<targets.size(); ++j) {
                std::string to_uuid_tmp;
                targets[j] >> to_uuid_tmp;

                if(to_uuid_tmp.find(UUID::namespace_separator) == to_uuid_tmp.npos) {
                    // legacy import
                    to_uuid_tmp.replace(to_uuid_tmp.find("_in_"), 1, UUID::namespace_separator);
                }

                UUID to_uuid = UUID::make_forced(to_uuid_tmp);

                Output* from = parent->getOutput(from_uuid);
                if(from == NULL) {
                    std::cerr << "cannot load connection, connector with uuid '" << from_uuid << "' doesn't exist." << std::endl;
                    continue;
                }

                try {
                    Node* target_box = graph_->findNodeForConnector(to_uuid);

                    Input* to = target_box->getInput(to_uuid);
                    if(!to) {
                        continue;
                    }

                    graph_->addConnection(Connection::Ptr(new Connection(from, to)));

                } catch(const std::exception& e) {
                    std::cerr << "cannot load connection: " << e.what() << std::endl;
                    continue;
                }

            }
        }
    }

    if(exists(doc, "fulcrums")) {
        const YAML::Node& fulcrums = doc["fulcrums"];
        apex_assert_hard(fulcrums.Type() == YAML::NodeType::Sequence);


        for(unsigned i = 0; i < fulcrums.size(); ++i) {
            const YAML::Node& fulcrum = fulcrums[i];
            apex_assert_hard(fulcrum.Type() == YAML::NodeType::Map);

            std::string from_uuid_tmp;
            fulcrum["from"] >> from_uuid_tmp;
            std::string to_uuid_tmp;
            fulcrum["to"] >> to_uuid_tmp;

            UUID from_uuid = UUID::make_forced(from_uuid_tmp);
            UUID to_uuid = UUID::make_forced(to_uuid_tmp);

            Output* from = dynamic_cast<Output*>(graph_->findConnector(from_uuid));
            if(from == NULL) {
                std::cerr << "cannot load fulcrum, connector with uuid '" << from_uuid << "' doesn't exist." << std::endl;
                continue;
            }

            Input* to = dynamic_cast<Input*>(graph_->findConnector(to_uuid));
            if(to == NULL) {
                std::cerr << "cannot load fulcrum, connector with uuid '" << to_uuid << "' doesn't exist." << std::endl;
                continue;
            }

            Connection::Ptr connection = graph_->getConnection(Connection::Ptr(new Connection(from, to)));

            std::vector< std::vector<double> > pts;
            fulcrum["pts"] >> pts;

            std::vector< std::vector<double> > handles;
            bool has_handle = YAML::exists(fulcrum, "handles");
            if(has_handle) {
                fulcrum["handles"] >> handles;
            }

            std::vector<int> types;
            if(exists(fulcrum, "types")) {
                fulcrum["types"] >> types;
            }

            int n = pts.size();
            for(int i = 0; i < n; ++i) {
                int type = (!types.empty()) ? types[i] : Fulcrum::CURVE;
                if(has_handle) {
                    QPointF in(handles[i][0], handles[i][1]);
                    QPointF out(handles[i][2], handles[i][3]);
                    connection->addFulcrum(i, QPointF(pts[i][0], pts[i][1]), type, in, out);
                } else {
                    connection->addFulcrum(i, QPointF(pts[i][0], pts[i][1]), type);
                }
            }
        }
    }
}
