/// HEADER
#include <csapex/core/graphio.h>

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_factory.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/trigger.h>
#include <csapex/signal/slot.h>
#include <csapex/model/fulcrum.h>
#include <csapex/utility/assert.h>
#include <csapex/model/node_state.h>
#include <csapex/model/connection.h>
#include <csapex/utility/yaml_node_builder.h>

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


void GraphIO::saveSettings(YAML::Node& doc)
{
    doc["uuid_map"] = graph_->uuids_;
}

void GraphIO::loadSettings(const YAML::Node &doc)
{
    if(doc["uuid_map"].IsDefined()) {
        graph_->uuids_ = doc["uuid_map"].as<std::map<std::string, int> >();
    }
}

void GraphIO::saveNodes(YAML::Node &yaml)
{
    foreach(NodeWorker* node, graph_->getAllNodeWorkers()) {
        try {
            YAML::Node yaml_node;
            node->getNodeState()->writeYaml(yaml_node);
            yaml["nodes"].push_back(yaml_node);
        } catch(const std::exception& e) {
            std::cerr << "cannot save state for node " << node->getUUID() << ": " << e.what() << std::endl;
            throw e;
        }
    }
}

void GraphIO::loadNode(const YAML::Node& doc)
{
    std::string type = doc["type"].as<std::string>();
    UUID uuid = doc["uuid"].as<UUID>();

    int x = doc["pos"][0].as<int>();
    int y = doc["pos"][1].as<int>();

    NodeWorker::Ptr node_worker = node_factory_->makeNode(type, uuid);
    if(!node_worker) {
        return;
    }

    try {
        NodeState::Ptr s = node_worker->getNodeState();
        s->readYaml(doc);
        node_worker->setNodeState(s);

    } catch(const std::exception& e) {
        std::cerr << "cannot load state for box " << uuid << ": " << typeid(e).name() << ", what=" << e.what() << std::endl;
    }
    if(x != 0 || y != 0) {
        node_worker->getNodeState()->setPos(QPoint(x,y));
    }
    graph_->addNode(node_worker);
}

void GraphIO::saveConnections(YAML::Node &yaml)
{
    foreach(NodeWorker* node, graph_->getAllNodeWorkers()) {
        if(!node->getAllOutputs().empty()) {
            foreach(Output* o, node->getAllOutputs()) {
                if(o->noTargets() == 0) {
                    continue;
                }
                YAML::Node connection;
                connection["uuid"] = o->getUUID();
                foreach(Input* i, o->getTargets()) {
                    connection["targets"].push_back(i->getUUID());
                }

                yaml["connections"].push_back(connection);
            }
            foreach(Trigger* o, node->getTriggers()) {
                if(o->noTargets() == 0) {
                    continue;
                }
                YAML::Node connection;
                connection["uuid"] = o->getUUID();
                foreach(Slot* i, o->getTargets()) {
                    connection["targets"].push_back(i->getUUID());
                }

                yaml["connections"].push_back(connection);
            }
        }
    }

    foreach(const Connection::Ptr& connection, graph_->connections_) {
        if(connection->getFulcrumCount() == 0) {
            continue;
        }

        YAML::Node fulcrum;
        fulcrum["from"] = connection->from()->getUUID();
        fulcrum["to"] = connection->to()->getUUID();

        Q_FOREACH(const Fulcrum::Ptr& f, connection->getFulcrums()) {
            YAML::Node pt;
            pt.push_back(f->pos().x());
            pt.push_back(f->pos().y());

            fulcrum["pts"].push_back(pt);
        }

        Q_FOREACH(const Fulcrum::Ptr& f, connection->getFulcrums()) {
            YAML::Node handle;
            handle.push_back(f->handleIn().x());
            handle.push_back(f->handleIn().y());
            handle.push_back(f->handleOut().x());
            handle.push_back(f->handleOut().y());

            fulcrum["handles"].push_back(handle);
        }

        Q_FOREACH(const Fulcrum::Ptr& f, connection->getFulcrums()) {
            fulcrum["types"].push_back(f->type());
        }

        yaml["fulcrums"].push_back(fulcrum);
    }
}

void GraphIO::loadConnections(const YAML::Node &doc)
{
    if(doc["connections"].IsDefined()) {
        const YAML::Node& connections = doc["connections"];
        apex_assert_hard(connections.Type() == YAML::NodeType::Sequence);

        for(unsigned i = 0; i < connections.size(); ++i) {
            const YAML::Node& connection = connections[i];
            apex_assert_hard(connection.Type() == YAML::NodeType::Map);

            std::string from_uuid_tmp = connection["uuid"].as<std::string>();
            if(from_uuid_tmp.find(UUID::namespace_separator) == from_uuid_tmp.npos) {
                // legacy import
                from_uuid_tmp.replace(from_uuid_tmp.find("_out_"), 1, UUID::namespace_separator);
            }

            UUID from_uuid = UUID::make_forced(from_uuid_tmp);

            NodeWorker* parent = NULL;
            try {
                parent = graph_->findNodeWorkerForConnector(from_uuid);

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
                std::string to_uuid_tmp = targets[j].as<std::string>();

                if(to_uuid_tmp.find(UUID::namespace_separator) == to_uuid_tmp.npos) {
                    // legacy import
                    to_uuid_tmp.replace(to_uuid_tmp.find("_in_"), 1, UUID::namespace_separator);
                }

                UUID to_uuid = UUID::make_forced(to_uuid_tmp);

                Connectable* from = parent->getOutput(from_uuid);
                if(from == NULL) {
                    from = parent->getTrigger(from_uuid);
                }
                if(from == NULL) {
                    std::cerr << "cannot load connection, connector with uuid '" << from_uuid << "' doesn't exist." << std::endl;
                    continue;
                }

                try {
                    NodeWorker* target = graph_->findNodeWorkerForConnector(to_uuid);

                    Connectable* to = target->getInput(to_uuid);
                    if(to == NULL) {
                        to = target->getSlot(to_uuid);
                    }
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

    if(doc["fulcrums"].IsDefined()) {
        const YAML::Node& fulcrums = doc["fulcrums"];
        apex_assert_hard(fulcrums.Type() == YAML::NodeType::Sequence);


        for(unsigned i = 0; i < fulcrums.size(); ++i) {
            const YAML::Node& fulcrum = fulcrums[i];
            apex_assert_hard(fulcrum.Type() == YAML::NodeType::Map);

            std::string from_uuid_tmp = fulcrum["from"].as<std::string>();
            std::string to_uuid_tmp = fulcrum["to"].as<std::string>();

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

            std::vector< std::vector<double> > pts = fulcrum["pts"].as<std::vector< std::vector<double> > >();

            std::vector< std::vector<double> > handles;
            bool has_handle = fulcrum["handles"].IsDefined();
            if(has_handle) {
                handles = fulcrum["handles"].as<std::vector< std::vector<double> > >();
            }

            std::vector<int> types;
            if(fulcrum["types"].IsDefined()) {
                types = fulcrum["types"].as<std::vector<int> >();
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
