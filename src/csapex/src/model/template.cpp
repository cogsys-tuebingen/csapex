/// HEADER
#include <csapex/model/template.h>

/// COMPONENT
#include <csapex/model/connectable.h>
#include <csapex/command/add_node.h>
#include <csapex/command/add_connector.h>
#include <csapex/command/add_connection.h>
#include <csapex/model/tag.h>

/// SYSTEM
#include <boost/foreach.hpp>
#include <QFile>

using namespace csapex;

const std::string Template::PARENT_PREFIX_PATTERN = "${parent}";

const QString Template::MIME = "csapex/template";

Template::Template(const std::string& name)
    : name_(name), next_connector_sub_id(0), locked(false)
{
}

std::string Template::addBox(const std::string &type, const QPoint &pos, NodeState::Ptr state)
{
    assert(!locked);
    assert(!type.empty());

    std::string uuid_tmp = PARENT_PREFIX_PATTERN + Graph::namespace_separator + tmp_graph.makeUUIDPrefix(type);
    UUID uuid = UUID::make(uuid_tmp);

    std::cerr << "adding box to template: " << uuid << " (" << type << ")" << std::endl;

    BoxTemplate box;
    box.type = type;
    box.pos = pos;
    box.uuid = uuid;
    box.state.copyFrom(state);
    box.state.parent = NULL;

    boxes.push_back(box);

    return uuid;
}

std::string Template::addConnector(const std::string &label, const std::string &type, bool input, bool forward)
{
    assert(!locked);

    std::string uuid_tmp = Connectable::makeUUIDPrefix(UUID::make_forced(PARENT_PREFIX_PATTERN), Connectable::TYPE_MISC, next_connector_sub_id);
    UUID uuid = UUID::make(uuid_tmp);

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

std::string Template::addConnection(const UUID &from_uuid, const UUID &to_uuid)
{
    assert(!locked);

    std::cerr << "adding connection to template: " << from_uuid << " -> " << to_uuid << std::endl;

    ConnectionTemplate c;
    c.from_uuid = from_uuid;
    c.to_uuid = to_uuid;

    connections.push_back(c);

    return "";
}

UUID Template::fillInTemplate(const UUID& uuid, const UUID& parent) {
    if(!uuid.contains(Template::PARENT_PREFIX_PATTERN)) {
        return uuid;
    } else {
        return uuid.replace(Template::PARENT_PREFIX_PATTERN, parent);
    }
}

void Template::createCommands(command::Meta* meta, const UUID& parent) const
{
    foreach (const Template::BoxTemplate& box, boxes) {
        UUID uuid = fillInTemplate(box.uuid, parent);
        NodeState::Ptr state(new NodeState(box.state));

        meta->add(Command::Ptr(new command::AddNode(box.type, box.pos, parent, uuid, state)));
    }

    foreach (const Template::ConnectorTemplate& c, connectors) {
        UUID uuid = fillInTemplate(c.uuid, parent);
        meta->add(Command::Ptr(new command::AddConnector(parent, c.label, c.type, c.input, uuid, c.forward)));
    }

    foreach (const Template::ConnectionTemplate& c, connections) {
        UUID f_uuid = fillInTemplate(c.from_uuid, parent);
        UUID t_uuid = fillInTemplate(c.to_uuid, parent);

        meta->add(Command::Ptr(new command::AddConnection(f_uuid, t_uuid)));
    }
}

void Template::setName(const std::string &name)
{
    name_ = name;
}

std::string Template::getName()
{
    return name_;
}


void Template::write(YAML::Emitter &emitter) const
{
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "name" << YAML::Value << name_;
    emitter << YAML::Key << "boxes" << YAML::Value << boxes;
    emitter << YAML::Key << "connectors" << YAML::Value << connectors;
    emitter << YAML::Key << "connections" << YAML::Value << connections;
    emitter << YAML::EndMap;
}

void Template::read(const YAML::Node &doc)
{
    doc["name"] >> name_;
    doc["boxes"] >> boxes;
    doc["connectors"] >> connectors;
    doc["connections"] >> connections;

    if(doc.FindValue("tags")) {
        std::vector<std::string> t;
        doc["tags"] >> t;

        BOOST_FOREACH(const std::string& tag, t) {
            Tag::createIfNotExists(tag);
            tags.push_back(Tag::get(tag));

        }
    }
    if(doc.FindValue("icon")) {
        std::string i;
        doc["icon"] >> i;
        std::string file = std::string(":/") + i;
        if(!QFile::exists(file.c_str())) {
            std::cerr << "error: cannot load the icon \"" << i << "\"" << std::endl;
            icon = QIcon(":/group.png");
        } else {
            icon = QIcon(file.c_str());
        }
    }
}

