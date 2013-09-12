#ifndef TEMPLATE_H
#define TEMPLATE_H

/// COMPONENT
#include <csapex/model/box.h>
#include <csapex/csapex_fwd.h>
#include <csapex/model/graph.h>
#include <csapex/manager/box_manager.h>

/// SYSTEM
#include <QPoint>

namespace csapex
{

class Template
{
    friend class TemplateManager;

public:
    typedef boost::shared_ptr<Template> Ptr;

    static const QString MIME;

public:
    static const std::string PARENT_PREFIX_PATTERN;

private:
    explicit Template(const std::string& name);

public:
    std::string addBox(const std::string& type, const QPoint& pos, Box::State::Ptr state);
    std::string addConnector(const std::string& label, const std::string& type, bool input, bool forward = false);
    std::string addConnection(const std::string& from_uuid, const std::string& to_uuid);

    void createCommands(command::Meta *meta, const std::string &parent) const;

    void setName(const std::string& name);
    std::string getName();

    static std::string fillInTemplate(const std::string& uuid, const std::string& parent);

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
        std::string uuid;
        Box::State state;

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
            node["uuid"] >> box.uuid;

            std::vector<int> pos;
            node["pos"] >> pos;
            box.pos = QPoint(pos[0], pos[1]);


            Box::Ptr tmp = BoxManager::instance().makeBox(box.type, box.uuid);
            box.state.parent = tmp.get();
            box.state.readYaml(node["state"]);
            box.state.parent = NULL;
        }
    };

    struct ConnectorTemplate {
        std::string type;
        std::string uuid;
        std::string label;
        bool input;
        bool forward;

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
            node["uuid"] >> connector.uuid;
            node["label"] >> connector.label;
            node["input"] >> connector.input;
            node["forward"] >> connector.forward;
        }
    };

    struct ConnectionTemplate {
        std::string from_uuid;
        std::string to_uuid;

        friend void operator << (YAML::Emitter& e, const ConnectionTemplate& connection) {
            e << YAML::Flow << YAML::BeginMap;
            e << YAML::Key << "from_uuid" << YAML::Value << connection.from_uuid;
            e << YAML::Key << "to_uuid" << YAML::Value << connection.to_uuid;
            e << YAML::EndMap;
        }
        friend void operator >> (const YAML::Node& node, ConnectionTemplate& connection) {
            assert(node.Type() == YAML::NodeType::Map);

            node["from_uuid"] >> connection.from_uuid;
            node["to_uuid"] >> connection.to_uuid;
        }
    };

    int next_connector_sub_id;
    Graph tmp_graph;

    std::vector<BoxTemplate> boxes;
    std::vector<ConnectorTemplate> connectors;
    std::vector<ConnectionTemplate> connections;

    bool locked;
};

}

#endif // TEMPLATE_H
