/// HEADER
#include <csapex/command/group_base.h>

/// PROJECT
#include <csapex/model/graph.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/event.h>
#include <csapex/signal/slot.h>
#include <csapex/model/connection.h>
#include <csapex/command/paste_graph.h>

using namespace csapex;
using namespace csapex::command;

GroupBase::GroupBase(const AUUID &graph_uuid, const std::string& type)
    : Meta(graph_uuid, type)
{

}

void GroupBase::setNodes(const std::vector<NodeHandle *> &n)
{
    nodes = n;

    node_set.clear();
    node_set.insert(nodes.begin(), nodes.end());
}

Point GroupBase::findTopLeftPoint() const
{
    Point insert_pos = nodes[0]->getNodeState()->getPos();
    for(NodeHandle* nh : nodes) {
        Point pos = nh->getNodeState()->getPos();
        if(pos.x < insert_pos.x) {
            insert_pos.x = pos.x;
        }
        if(pos.y < insert_pos.y) {
            insert_pos.y = pos.y;
        }
    }

    return insert_pos;
}



void GroupBase::analyzeConnections(Graph* graph)
{
    connections_going_in.clear();
    connections_going_out.clear();
    signals_going_in.clear();
    signals_going_out.clear();

    for(NodeHandle* nh : nodes) {
        for(const InputPtr& input : nh->getExternalInputs()) {
            for(const ConnectionPtr& connection : input->getConnections()) {
                OutputPtr output = connection->from();
                apex_assert_hard(output);

                NodeHandle* source = graph->findNodeHandleForConnector(output->getUUID());
                apex_assert_hard(source);

                ConnectionInformation c(output.get(), input.get(), output->getType(), connection->isActive());
                if(node_set.find(source) == node_set.end()) {
                    // coming in
                    connections_going_in.push_back(c);
                }
            }
        }
        for(const OutputPtr& output : nh->getExternalOutputs()) {
            for(const ConnectionPtr& connection : output->getConnections()) {
                InputPtr input = connection->to();
                apex_assert_hard(input);

                NodeHandle* target = graph->findNodeHandleForConnector(input->getUUID());
                apex_assert_hard(target);

                if(node_set.find(target) == node_set.end()) {
                    // going out
                    ConnectionInformation c(output.get(), input.get(), input->getType(), connection->isActive());
                    connections_going_out.push_back(c);
                }
            }
        }

        for(const SlotPtr& slot : nh->getExternalSlots()) {
            for(const ConnectionPtr& connection : slot->getConnections()) {
                OutputPtr output = connection->from();
                apex_assert_hard(output);

                NodeHandle* target = graph->findNodeHandleForConnector(output->getUUID());
                apex_assert_hard(target);

                if(node_set.find(target) == node_set.end()) {
                    // going out
                    ConnectionInformation c(output.get(), slot.get(), output->getType(), connection->isActive());
                    signals_going_in.push_back(c);
                }
            }
        }
        for(const EventPtr& trigger : nh->getExternalEvents()) {
            for(const ConnectionPtr& connection : trigger->getConnections()) {
                InputPtr input = connection->to();
                apex_assert_hard(input);

                NodeHandle* source = graph->findNodeHandleForConnector(input->getUUID());
                apex_assert_hard(source);

                ConnectionInformation c(trigger.get(), input.get(), input->getType(), connection->isActive());
                if(node_set.find(source) == node_set.end()) {
                    // coming in
                    signals_going_out.push_back(c);
                }
            }
        }
    }
}

void GroupBase::pasteSelection(AUUID sub_graph_auuid)
{
    std::shared_ptr<PasteGraph> paste(new command::PasteGraph(sub_graph_auuid, serialized_snippet_, insert_pos));
    executeCommand(paste);
    add(paste);

    old_uuid_to_new = paste->getMapping();
}
