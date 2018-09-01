/// HEADER
#include <csapex/command/add_connection.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/node_constructor.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/model/graph_facade.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/direct_connection.h>
#include <csapex/command/command_serializer.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/serialization/io/csapex_io.h>

using namespace csapex;
using namespace csapex::command;

CSAPEX_REGISTER_COMMAND_SERIALIZER(AddConnection)

AddConnection::AddConnection(const AUUID& parent_uuid, const UUID& from_uuid, const UUID& to_uuid, bool active)
  : CommandImplementation(parent_uuid), from_uuid(from_uuid), to_uuid(to_uuid), active(active)
{
}

std::string AddConnection::getDescription() const
{
    return std::string("added a connection between ") + from_uuid.getFullName() + " and " + to_uuid.getFullName();
}

bool AddConnection::doUndo()
{
    GraphImplementationPtr graph = getGraph();

    ConnectorPtr f = graph->findConnector(from_uuid);
    ConnectorPtr t = graph->findConnector(to_uuid);

    apex_assert_hard((f->isOutput() && t->isInput()));

    graph->deleteConnection(graph->getConnection(from_uuid, to_uuid));

    return true;
}

bool AddConnection::doRedo()
{
    return doExecute();
}

bool AddConnection::doExecute()
{
    GraphImplementationPtr graph = getGraph();

    OutputPtr f = graph->findTypedConnector<Output>(from_uuid);
    InputPtr t = graph->findTypedConnector<Input>(to_uuid);

    apex_assert_hard(f);
    apex_assert_hard(t);
    apex_assert_hard((f->isOutput() && t->isInput()));

    ConnectionPtr c = DirectConnection::connect(f, t);
    apex_assert_hard(c);
    c->setActive(active);

    return graph->addConnection(c);
}

void AddConnection::serialize(SerializationBuffer& data, SemanticVersion& version) const
{
    Command::serialize(data, version);

    data << from_uuid;
    data << to_uuid;
    data << active;
}

void AddConnection::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    Command::deserialize(data, version);

    data >> from_uuid;
    data >> to_uuid;
    data >> active;
}
