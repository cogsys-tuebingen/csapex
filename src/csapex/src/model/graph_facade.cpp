/// HEADER
#include <csapex/model/graph_facade.h>

/// COMPONENT
#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/connection.h>
#include <csapex/core/settings.h>
#include <csapex/scheduling/task_generator.h>
#include <csapex/model/node_runner.h>
#include <csapex/scheduling/executor.h>
#include <csapex/msg/bundled_connection.h>
#include <csapex/model/connectable.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>

using namespace csapex;

GraphFacade::GraphFacade(Executor &executor, Graph* graph)
    : graph_(graph), executor_(executor)
{
    connections_.push_back(graph->nodeAdded.connect([this](NodeHandlePtr n) {
                               NodeWorkerPtr nw = std::make_shared<NodeWorker>(n);

                               TaskGeneratorPtr runner = std::make_shared<NodeRunner>(nw);
                               generators_[n->getUUID()] = runner;
                               executor_.add(runner.get());
                               generatorAdded(runner);

                               nodeAdded(n);
                               nodeWorkerAdded(nw);

                               nw->checkParameters();
                               nw->panic.connect(panic);

                           }));

    connections_.push_back(graph->nodeRemoved.connect([this](NodeHandlePtr n) {
                               TaskGeneratorPtr runner = generators_[n->getUUID()];
                               generators_.erase(n->getUUID());
                               executor_.remove(runner.get());
                               generatorRemoved(runner);

                               NodeWorkerPtr nw = node_workers_[n.get()];
                               nodeWorkerRemoved(nw);
                               node_workers_.erase(n.get());
                               nodeRemoved(n);
                           }));
}

GraphFacade::~GraphFacade()
{
    for(auto& connection : connections_) {
        connection.disconnect();
    }
    connections_.clear();
}

Graph* GraphFacade::getGraph()
{
    return graph_;
}

void GraphFacade::addNode(NodeHandlePtr node)
{
    graph_->addNode(node);
}

ConnectionPtr GraphFacade::connect(OutputPtr output, InputPtr input, OutputTransition *ot, InputTransition *it)
{
    return GraphFacade::connect(output.get(), input.get(), ot, it);
}

ConnectionPtr GraphFacade::connect(Output *output, Input *input, OutputTransition *ot, InputTransition *it)
{
    auto c = BundledConnection::connect(output, input, ot, it);
    graph_->addConnection(c);
    return c;
}

ConnectionPtr GraphFacade::connect(NodeHandlePtr output, int output_id,
                                   NodeHandlePtr input, int input_id)
{
    return GraphFacade::connect(output.get(), output_id, input.get(), input_id);
}

ConnectionPtr GraphFacade::connect(NodeHandle *output, int output_id,
                                   NodeHandle *input, int input_id)
{
    Output* o = output->getOutput(Connectable::makeUUID_forced(output->getUUID(), "out", output_id));
    if(!o) {
        throw std::logic_error(output->getUUID().getFullName() +
                               " does not have an output with the index " +
                               std::to_string(output_id));
    }
    Input* i = input->getInput(Connectable::makeUUID_forced(input->getUUID(), "in", input_id));
    if(!i) {
        throw std::logic_error(input->getUUID().getFullName() +
                               " does not have an input with the label " +
                               std::to_string(input_id));
    }

    auto c = BundledConnection::connect(o, i, output->getOutputTransition(), input->getInputTransition());
    graph_->addConnection(c);
    return c;
}

ConnectionPtr GraphFacade::connect(NodeHandlePtr output, const std::string& output_id,
                                   NodeHandlePtr input, const std::string& input_id)
{
    return GraphFacade::connect(output.get(), output_id, input.get(), input_id);
}

ConnectionPtr GraphFacade::connect(NodeHandle *output, const std::string& output_id,
                                   NodeHandle *input, const std::string& input_id)
{
    Output* o = nullptr;
    for(auto out : output->getAllOutputs()) {
        Output* out_ptr = out.get();
        if(out_ptr->getLabel() == output_id) {
            o = out_ptr;
            break;
        }
    }
    if(!o) {
        throw std::logic_error(output->getUUID().getFullName() +
                               " does not have an output with the label " +
                               output_id);
    }
    Input* i = nullptr;
    for(auto in : input->getAllInputs()) {
        Input* in_ptr = in.get();
        if(in_ptr->getLabel() == input_id) {
            i = in_ptr;
            break;
        }
    }
    if(!i) {
        throw std::logic_error(input->getUUID().getFullName() +
                               " does not have an input with the label " +
                               input_id);
    }
    auto c = BundledConnection::connect(o, i, output->getOutputTransition(), input->getInputTransition());
    graph_->addConnection(c);
    return c;
}

ConnectionPtr GraphFacade::connect(const UUID& output_id, const UUID& input_id)
{
    NodeHandle* output = graph_->findNodeHandleForConnector(output_id);
    NodeHandle* input = graph_->findNodeHandleForConnector(input_id);
    apex_assert_hard(output);
    apex_assert_hard(input);
    Output* o = output->getOutput(output_id);
    Input* i = input->getInput(input_id);
    apex_assert_hard(o);
    apex_assert_hard(i);
    auto c = BundledConnection::connect(o, i, output->getOutputTransition(), input->getInputTransition());
    graph_->addConnection(c);
    return c;
}

TaskGenerator* GraphFacade::getTaskGenerator(const UUID &uuid)
{
    return generators_.at(uuid).get();
}

void GraphFacade::reset()
{
    stop();
    graph_->clear();
    for(auto& gen: generators_) {
        generatorRemoved(gen.second);
    }
    generators_.clear();
}

bool GraphFacade::isPaused() const
{
    return executor_.isPaused();
}

void GraphFacade::pauseRequest(bool pause)
{
    if(executor_.isPaused() == pause) {
        return;
    }

    executor_.setPause(pause);

    paused(pause);
}


void GraphFacade::stop()
{
    for(NodeHandle* nw : graph_->getAllNodeHandles()) {
        nw->stop();
    }

    executor_.stop();

    stopped();
}

void GraphFacade::clearBlock()
{
    executor_.clear();
}
