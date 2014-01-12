#ifndef TEMPLATE_H
#define TEMPLATE_H

/// COMPONENT
#include <csapex/view/box.h>
#include <csapex/csapex_fwd.h>
#include <csapex/model/graph.h>
#include <csapex/manager/box_manager.h>
#include <csapex/model/node_state.h>
#include <csapex/model/tag.h>

/// SYSTEM
#include <QPoint>

namespace csapex
{

class Template
{
    friend class TemplateManager;
    friend class TemplateConstructor;

public:
    typedef boost::shared_ptr<Template> Ptr;

    static const QString MIME;

public:
    static const std::string PARENT_PREFIX_PATTERN;

private:
    explicit Template(const std::string& name);

public:
    std::string addBox(const std::string& type, const QPoint& pos, NodeStatePtr state);
    std::string addConnector(const std::string& label, const std::string& type, bool input, bool forward = false);
    std::string addConnection(const UUID &from_uuid, const UUID &to_uuid);

    void createCommands(command::Meta *meta, const UUID &parent) const;

    void setName(const std::string& name);
    std::string getName();

    static UUID fillInTemplate(const UUID& uuid, const UUID &parent);

    void write(YAML::Emitter& emitter) const;
    void read(const YAML::Node& doc);

public:
    friend YAML::Emitter& operator << (YAML::Emitter& e, const Template& p) {
        p.write(e);
        return e;
    }
    friend YAML::Emitter& operator << (YAML::Emitter& e, Template::Ptr p) {
        p->write(e);
        return e;
    }

    friend void operator >> (const YAML::Node& node, csapex::Template& value) {
        value.read(node);
    }
    friend void operator >> (const YAML::Node& node, csapex::Template::Ptr& value) {
        value.reset(new csapex::Template(""));
        value->read(node);
    }


private:
    std::string name_;

    struct BoxTemplate {
        std::string type;
        QPoint pos;
        UUID uuid;
        NodeState state;

        BoxTemplate()
            : uuid(UUID::NONE), state(NULL)
        {
        }

        friend void operator << (YAML::Emitter& e, const BoxTemplate& box) {
            e << YAML::BeginMap;
            e << YAML::Key << "type" << YAML::Value << box.type;
            e << YAML::Key << "uuid" << YAML::Value << box.uuid;
            e << YAML::Key << "pos" << YAML::Value << YAML::Flow << YAML::BeginSeq <<  box.pos.x() << box.pos.y() << YAML::EndSeq;
            e << YAML::Key << "state" << YAML::Value;
            box.state.writeYaml(e);
            e << YAML::EndMap;
        }
        friend void operator >> (const YAML::Node& node, BoxTemplate& box) {
            assert(node.Type() == YAML::NodeType::Map);

            node["type"] >> box.type;

            std::string uuid_tmp;
            node["uuid"] >> uuid_tmp;
            box.uuid = UUID::make_forced(uuid_tmp);

            std::vector<int> pos;
            node["pos"] >> pos;
            box.pos = QPoint(pos[0], pos[1]);


            NodePtr tmp = BoxManager::instance().makeNode(box.type, box.uuid);
            box.state.parent = tmp.get();
            box.state.readYaml(node);
            box.state.parent = NULL;
        }
    };

    struct ConnectorTemplate {
        std::string type;
        UUID uuid;
        std::string label;
        bool input;
        bool forward;

        ConnectorTemplate()
            : uuid(UUID::NONE)
        {
        }

        friend void operator << (YAML::Emitter& e, const ConnectorTemplate& connector) {
            e << YAML::Flow << YAML::BeginMap;
            e << YAML::Key << "type" << YAML::Value << connector.type;
            e << YAML::Key << "uuid" << YAML::Value << connector.uuid;
            e << YAML::Key << "label" << YAML::Value << connector.label;
            e << YAML::Key << "input" << YAML::Value << connector.input;
            e << YAML::Key << "forward" << YAML::Value << connector.forward;
            e << YAML::EndMap;
        }
        friend void operator >> (const YAML::Node& node, ConnectorTemplate& connector) {
            assert(node.Type() == YAML::NodeType::Map);

            node["type"] >> connector.type;
            node["label"] >> connector.label;
            node["input"] >> connector.input;
            node["forward"] >> connector.forward;

            std::string uuid_tmp;
            node["uuid"] >> uuid_tmp;
            connector.uuid = UUID::make_forced(uuid_tmp);
        }
    };

    struct ConnectionTemplate {
        UUID from_uuid;
        UUID to_uuid;

        ConnectionTemplate()
            : from_uuid(UUID::NONE), to_uuid(UUID::NONE)
        {
        }

        friend void operator << (YAML::Emitter& e, const ConnectionTemplate& connection) {
            e << YAML::Flow << YAML::BeginMap;
            e << YAML::Key << "from_uuid" << YAML::Value << connection.from_uuid;
            e << YAML::Key << "to_uuid" << YAML::Value << connection.to_uuid;
            e << YAML::EndMap;
        }
        friend void operator >> (const YAML::Node& node, ConnectionTemplate& connection) {
            assert(node.Type() == YAML::NodeType::Map);

            std::string from, to;

            node["from_uuid"] >> from;
            node["to_uuid"] >> to;

            connection.from_uuid = UUID::make_forced(from);
            connection.to_uuid = UUID::make_forced(to);
        }
    };

    int next_connector_sub_id;
    Graph tmp_graph;

    std::vector<BoxTemplate> boxes;
    std::vector<ConnectorTemplate> connectors;
    std::vector<ConnectionTemplate> connections;
    std::vector<Tag> tags;
    QIcon icon;

    bool locked;
};

}

#endif // TEMPLATE_H
