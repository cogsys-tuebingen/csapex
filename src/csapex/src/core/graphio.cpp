/// HEADER
#include <csapex/core/graphio.h>

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_factory.h>
#include <csapex/model/connection.h>
#include <csapex/model/graph_worker.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/trigger.h>
#include <csapex/signal/slot.h>
#include <csapex/model/fulcrum.h>
#include <csapex/utility/assert.h>
#include <csapex/model/node_state.h>
#include <csapex/model/connection.h>
#include <csapex/utility/yaml_node_builder.h>
#include <csapex/core/serialization.h>

/// SYSTEM
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <sys/types.h>

using namespace csapex;

GraphIO::GraphIO(GraphWorker *graph, NodeFactory* node_factory)
    : graph_worker_(graph), node_factory_(node_factory)
{
}


void GraphIO::saveSettings(YAML::Node& doc)
{
    doc["uuid_map"] = graph_worker_->getGraph()->uuids_;
}

void GraphIO::loadSettings(const YAML::Node &doc)
{
    if(doc["uuid_map"].IsDefined()) {
        graph_worker_->getGraph()->uuids_ = doc["uuid_map"].as<std::map<std::string, int> >();
    }
}

void GraphIO::saveNodes(YAML::Node &yaml)
{
    for(NodeWorker* node : graph_worker_->getGraph()->getAllNodeWorkers()) {
        try {
            YAML::Node yaml_node;
            serializeNode(yaml_node, node);
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

    NodeWorker::Ptr node_worker = node_factory_->makeNode(type, uuid);
    if(!node_worker) {
        return;
    }

    try {
        deserializeNode(doc, node_worker.get());

    } catch(const std::exception& e) {
        std::cerr << "cannot load state for box " << uuid << ": " << typeid(e).name() << ", what=" << e.what() << std::endl;
    }
    node_worker->pause(graph_worker_->isPaused());
    graph_worker_->getGraph()->addNode(node_worker);
}

void GraphIO::saveConnections(YAML::Node &yaml)
{
    for(NodeWorker* node : graph_worker_->getGraph()->getAllNodeWorkers()) {
        if(!node->getAllOutputs().empty()) {
            for(Output* o : node->getAllOutputs()) {
                if(o->countConnections() == 0) {
                    continue;
                }
                YAML::Node connection;
                connection["uuid"] = o->getUUID();
                for(ConnectionPtr c : o->getConnections()) {
                    connection["targets"].push_back(c->to()->getUUID());
                }

                yaml["connections"].push_back(connection);
            }
            for(Trigger* o : node->getTriggers()) {
                if(o->noTargets() == 0) {
                    continue;
                }
                YAML::Node connection;
                connection["uuid"] = o->getUUID();
                for(Slot* i : o->getTargets()) {
                    connection["targets"].push_back(i->getUUID());
                }

                yaml["connections"].push_back(connection);
            }
        }
    }

    for(const ConnectionPtr& connection : graph_worker_->getGraph()->connections_) {
        if(connection->getFulcrumCount() == 0) {
            continue;
        }

        YAML::Node fulcrum;
        fulcrum["from"] = connection->from()->getUUID();
        fulcrum["to"] = connection->to()->getUUID();

        for(const Fulcrum::Ptr& f : connection->getFulcrums()) {
            YAML::Node pt;
            pt.push_back(f->pos().x);
            pt.push_back(f->pos().y);

            fulcrum["pts"].push_back(pt);
        }

        for(const Fulcrum::Ptr& f : connection->getFulcrums()) {
            YAML::Node handle;
            handle.push_back(f->handleIn().x);
            handle.push_back(f->handleIn().y);
            handle.push_back(f->handleOut().x);
            handle.push_back(f->handleOut().y);

            fulcrum["handles"].push_back(handle);
        }

        for(const Fulcrum::Ptr& f : connection->getFulcrums()) {
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

            NodeWorker* parent = nullptr;
            try {
                parent = graph_worker_->getGraph()->findNodeWorkerForConnector(from_uuid);

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
                if(from == nullptr) {
                    from = parent->getTrigger(from_uuid);
                }
                if(from == nullptr) {
                    std::cerr << "cannot load connection, connector with uuid '" << from_uuid << "' doesn't exist." << std::endl;
                    continue;
                }

                try {
                    NodeWorker* target = graph_worker_->getGraph()->findNodeWorkerForConnector(to_uuid);

                    Connectable* to = target->getInput(to_uuid);
                    if(to == nullptr) {
                        to = target->getSlot(to_uuid);
                    }
                    if(!to) {
                        continue;
                    }

                    graph_worker_->getGraph()->addConnection(ConnectionPtr(new Connection(from, to)));

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

            Output* from = dynamic_cast<Output*>(graph_worker_->getGraph()->findConnector(from_uuid));
            if(from == nullptr) {
                std::cerr << "cannot load fulcrum, connector with uuid '" << from_uuid << "' doesn't exist." << std::endl;
                continue;
            }

            Input* to = dynamic_cast<Input*>(graph_worker_->getGraph()->findConnector(to_uuid));
            if(to == nullptr) {
                std::cerr << "cannot load fulcrum, connector with uuid '" << to_uuid << "' doesn't exist." << std::endl;
                continue;
            }

            ConnectionPtr connection = graph_worker_->getGraph()->getConnection(from, to);

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
                    Point in(handles[i][0], handles[i][1]);
                    Point out(handles[i][2], handles[i][3]);
                    connection->addFulcrum(i, Point(pts[i][0], pts[i][1]), type, in, out);
                } else {
                    connection->addFulcrum(i, Point(pts[i][0], pts[i][1]), type);
                }
            }
        }
    }
}

void GraphIO::serializeNode(YAML::Node& doc, NodeWorker* node_worker)
{
    node_worker->getNodeState()->writeYaml(doc);

    // hook for nodes to serialize
    Serialization::instance().serialize(*node_worker->getNode(), doc);
}

void GraphIO::deserializeNode(const YAML::Node& doc, NodeWorker* node_worker)
{
    NodeState::Ptr s = node_worker->getNodeState();
    s->readYaml(doc);

    int x = doc["pos"][0].as<int>();
    int y = doc["pos"][1].as<int>();

    if(x != 0 || y != 0) {
        s->setPos(QPoint(x,y));
    }
    node_worker->setNodeState(s);

    // hook for nodes to deserialize
    Serialization::instance().deserialize(*node_worker->getNode(), doc);
}
