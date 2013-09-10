/// HEADER
#include <csapex/sub_graph_template.h>

/// COMPONENT
#include <csapex/connector.h>
#include <csapex/command_add_box.h>
#include <csapex/command_add_connector.h>
#include <csapex/command_add_connection.h>
#include <csapex/command_instanciate_subgraph_template.h>

using namespace csapex;

const std::string SubGraphTemplate::PARENT_PREFIX_PATTERN = "${parent}";

const QString SubGraphTemplate::MIME = "csapex/template";

SubGraphTemplate::SubGraphTemplate(const std::string& name)
    : name_(name), next_connector_sub_id(0), locked(false)
{
}

std::string SubGraphTemplate::addBox(const std::string &type, const std::string &templ, const QPoint &pos, Box::State::Ptr state)
{
    assert(!locked);

    std::string uuid = PARENT_PREFIX_PATTERN + Graph::namespace_separator + tmp_graph.makeUUID(type);

    std::cerr << "adding box " << (templ.empty() ? "" : ( std::string("(") + templ + ")")) << " to template: " << uuid << " (" << type << ")" << std::endl;

    BoxTemplate box;
    box.type = type;
    box.pos = pos;
    box.uuid = uuid;
    box.state.copyFrom(state);
    box.state.parent = NULL;
    box.templ = templ;

    boxes.push_back(box);

    return uuid;
}

std::string SubGraphTemplate::addConnector(const std::string &label, const std::string &type, bool input, bool forward)
{
    assert(!locked);

    std::string uuid = Connector::makeUUID(PARENT_PREFIX_PATTERN, Connector::TYPE_MISC, next_connector_sub_id);

    std::cerr << "adding connector to template: " << uuid << std::endl;

    ++next_connector_sub_id;

    ConnectorTemplate c;
    c.label = label;
    c.type = type;
    c.input = input;
    c.uuid = uuid;
    c.forward = forward;

    connectors.push_back(c);

    return uuid;
}

std::string SubGraphTemplate::addConnection(const std::string &from_uuid, const std::string &to_uuid)
{
    assert(!locked);

    std::cerr << "adding connection to template: " << from_uuid << " -> " << to_uuid << std::endl;

    ConnectionTemplate c;
    c.from_uuid = from_uuid;
    c.to_uuid = to_uuid;

    connections.push_back(c);

    return "";
}

std::string SubGraphTemplate::fillInTemplate(const std::string& uuid, const std::string& parent) {
    size_t pos = uuid.find(SubGraphTemplate::PARENT_PREFIX_PATTERN);
    if(pos == uuid.npos) {
        return uuid;
    } else {
        std::string result = uuid;
        return result.replace(pos, SubGraphTemplate::PARENT_PREFIX_PATTERN.length(), parent);
    }
}

void SubGraphTemplate::createCommands(command::Meta* meta, const std::string& parent) const
{
    foreach (const SubGraphTemplate::BoxTemplate& box, boxes) {
        std::string uuid = fillInTemplate(box.uuid, parent);
        Box::State::Ptr state(new Box::State(box.state));

        if(box.templ.empty()) {
            meta->add(Command::Ptr(new command::AddBox(box.type, box.pos, parent, uuid, state)));
        } else {
            meta->add(Command::Ptr(new command::InstanciateSubGraphTemplate(box.templ, uuid, box.pos)));
        }
    }

    foreach (const SubGraphTemplate::ConnectorTemplate& c, connectors) {
        std::string uuid = fillInTemplate(c.uuid, parent);
        meta->add(Command::Ptr(new command::AddConnector(parent, c.label, c.type, c.input, uuid, c.forward)));
    }

    foreach (const SubGraphTemplate::ConnectionTemplate& c, connections) {
        std::string f_uuid = fillInTemplate(c.from_uuid, parent);
        std::string t_uuid = fillInTemplate(c.to_uuid, parent);

        meta->add(Command::Ptr(new command::AddConnection(f_uuid, t_uuid)));
    }
}

void SubGraphTemplate::setName(const std::string &name)
{
    name_ = name;
}

std::string SubGraphTemplate::getName()
{
    return name_;
}


void SubGraphTemplate::write(YAML::Emitter &emitter) const
{
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "name" << YAML::Value << name_;
    emitter << YAML::Key << "boxes" << YAML::Value << boxes;
    emitter << YAML::Key << "connectors" << YAML::Value << connectors;
    emitter << YAML::Key << "connections" << YAML::Value << connections;
    emitter << YAML::EndMap;
}

void SubGraphTemplate::read(const YAML::Node &doc)
{
    doc["name"] >> name_;
    doc["boxes"] >> boxes;
    doc["connectors"] >> connectors;
    doc["connections"] >> connections;
}

