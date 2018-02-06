/// HEADER
#include <csapex/model/subprocess_node_worker.h>

/// COMPONENT
#include <csapex/factory/node_factory_impl.h>
#include <csapex/model/generic_state.h>
#include <csapex/model/graph/vertex.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/node_state.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/end_of_sequence_message.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/input.h>
#include <csapex/msg/input_transition.h>
#include <csapex/msg/io.h>
#include <csapex/msg/marker_message.h>
#include <csapex/msg/no_message.h>
#include <csapex/msg/output_transition.h>
#include <csapex/msg/static_output.h>
#include <csapex/param/trigger_parameter.h>
#include <csapex/profiling/profiler_impl.h>
#include <csapex/profiling/timer.h>
#include <csapex/signal/event.h>
#include <csapex/signal/slot.h>
#include <csapex/serialization/serialization_buffer.h>
#include <csapex/utility/debug.h>
#include <csapex/utility/delegate_bind.h>
#include <csapex/utility/exceptions.h>
#include <csapex/utility/thread.h>
#include <csapex/serialization/packet_serializer.h>

/// SYSTEM
#include <thread>
#include <iostream>
#include <cstdlib>
#include <scoped_allocator>
#include <boost/interprocess/exceptions.hpp>

using namespace csapex;
using namespace boost::interprocess;

//ros import, e.g., cannot be subprocessed .... -> ros init

SubprocessNodeWorker::SubprocessNodeWorker(NodeHandlePtr node_handle)
    : NodeWorker(node_handle),
      pid_(-1),
      subprocess_(node_handle->getUUID().getFullName())
{
}

void SubprocessNodeWorker::initialize()
{
    // ... generalize.......
    connection_types::GenericVectorMessage::registerType<connection_types::GenericValueMessage<int>>();
    connection_types::GenericVectorMessage::registerType<connection_types::GenericValueMessage<int>>(connection_types::type<connection_types::GenericValueMessage<int>>::name());

    pid_ = subprocess_.fork([this](){
        runSubprocessLoop();
    });

    NodeWorker::initialize();
}


void SubprocessNodeWorker::runSubprocessLoop()
{
    NodePtr node = getNode();

    try {
        observe(node->getParameterState()->parameter_changed, [&](param::Parameter* p){
            changed_parameters_.push_back(p);
        });

        while(subprocess_.isActive()) {

            // wait for a message
            SubprocessChannel::Message msg = subprocess_.in.read();
            if(msg.type == SubprocessChannel::MessageType::SHUTDOWN) {
                return;
            }

            switch(msg.type)
            {
            case SubprocessChannel::MessageType::PARAMETER_UPDATE:
                handleParameterUpdate(msg);
                break;

            case SubprocessChannel::MessageType::PROCESS:
                handleProcessChild(msg);
                break;

            default:
                break;
            }
        }

        std::quick_exit(0);

    } catch(const boost::interprocess::interprocess_exception& e) {
        std::cout << "interprocess exception " << getUUID() << " >> error: " << e.what() << std::endl;
        std::cout << "native error: " << e.get_native_error() << std::endl;
        std::cout << "error code:   " << e.get_error_code() << std::endl;
    } catch(const std::exception& e) {
        std::cout << "subprocess " << getUUID() << " >> error: " << e.what() << std::endl;
    }
}


void SubprocessNodeWorker::handleProcessChild(const SubprocessChannel::Message& msg)
{
    NodePtr node = getNode();

    YAML::Emitter result_emitter;

    try {
        if(msg.data) {
            YAML::Node yaml = YAML::Load(msg.toString());

            for(const YAML::Node& node : yaml) {
                UUID uuid = node["uuid"].as<UUID>();
                auto msg = MessageSerializer::deserializeMessage(node["data"]);

                InputPtr input = node_handle_->getInput(uuid);
                input->setToken(std::make_shared<Token>(msg));
            };
        }

        node->process(*node_handle_, *node);

        // send parameter updates
        for(param::Parameter* parameter : changed_parameters_) {
            transmitParameter(getNode()->getParameter(parameter->name()));
        }

        changed_parameters_.clear();


        // send result
        YAML::Node yaml(YAML::NodeType::Sequence);

        for(OutputPtr& output : node_handle_->getExternalOutputs()) {
            auto msg = output->getAddedToken();

            if(msg) {
                YAML::Node node(YAML::NodeType::Map);
                node["uuid"] = output->getUUID();
                // TODO serialize token! (+ activity, ...)
                node["data"] = MessageSerializer::serializeMessage(*msg->getTokenData());

                YAML::Emitter emitter;
                emitter << node;

                yaml.push_back(node);
            }
        }

        result_emitter << yaml;

    } catch(const std::exception& e) {
        std::cout << getUUID() << " >> error: " << e.what() << std::endl;
    }


    subprocess_.out.write({SubprocessChannel::MessageType::PROCESS, result_emitter.c_str()});
}

SubprocessNodeWorker::~SubprocessNodeWorker()
{
    stopObserving();
}


void SubprocessNodeWorker::handleParameterUpdate(const SubprocessChannel::Message &msg)
{
    if(msg.data) {
        NodePtr node = getNode();

        SerializationBuffer buffer((const uint8_t*) msg.data, msg.length);

        param::ParameterPtr p = std::dynamic_pointer_cast<param::Parameter>(PacketSerializer::deserializePacket(buffer));

        param::ParameterPtr existing = node->getParameter(p->name());
        existing->setValueFrom(*p);
    }
}

bool SubprocessNodeWorker::handleProcessParent(const SubprocessChannel::Message& msg)
{
    if(msg.data) {
        YAML::Node yaml = YAML::Load(msg.toString());

        for(const YAML::Node& node : yaml) {
            UUID uuid = node["uuid"].as<UUID>();
            auto msg = MessageSerializer::deserializeMessage(node["data"]);

            OutputPtr output = node_handle_->getOutput(uuid);
            YAML::Emitter emitter;
            emitter << node["data"];
            msg::publish(output.get(), msg);
        };
    }

    return true;
}

void SubprocessNodeWorker::processNode()
{
    apex_assert(pid_ != -1);

    std::unique_lock<std::recursive_mutex> lock(current_exec_mode_mutex_);

    NodePtr node = node_handle_->getNode().lock();
    apex_assert_hard(node);

    bool sync = !node->isAsynchronous();

    try {
        apex_assert_hard(node->getNodeHandle());
        if(sync) {
            apex_assert_msg(pid_ != 0, "processNode called in subprocess");

            // APEX
            YAML::Node yaml(YAML::NodeType::Sequence);

            for(InputPtr& input : node_handle_->getExternalInputs()) {
                if(msg::hasMessage(input.get())) {
                    auto msg = msg::getMessage(input.get());

                    if(msg) {
                        YAML::Node node(YAML::NodeType::Map);
                        node["uuid"] = input->getUUID();
                        // TODO serialize token! (+ activity, ...)
                        node["data"] = MessageSerializer::serializeMessage(*msg);
                        yaml.push_back(node);
                    }
                }
            }

            YAML::Emitter emitter;
            emitter << yaml;

            subprocess_.in.write({SubprocessChannel::MessageType::PROCESS, emitter.c_str()});

            bool done_processing = false;

            // wait for the end of processing

            while(!done_processing) {
                SubprocessChannel::Message msg = subprocess_.out.read();
                switch(msg.type)
                {
                case SubprocessChannel::MessageType::PARAMETER_UPDATE:
                    handleParameterUpdate(msg);
                    break;

                case SubprocessChannel::MessageType::PROCESS:
                    done_processing = handleProcessParent(msg);
                    break;

                default:
                    node->awarn << "Unhandled subprocess message: " << (int) msg.type << std::endl;
                    break;
                }
            }

        } else {
            // TODO: Implement this in subprocess, for now just call it directly
            try {
//                throw std::runtime_error("foo");
                //TRACE node->ainfo << "process async" << std::endl;
                node->process(*node_handle_, *node, [this, node](ProcessingFunction f) {
                    node_handle_->execution_requested([this, f, node]() {
                        if(f) {
                            f(*node_handle_, *node);
                        }
                        //TRACE getNode()->ainfo << "async process done -> finish processing" << std::endl;
                        finishProcessing();
                    });
                });
            } catch(...) {
                // if flow is aborted -> call continuation anyway
                //TRACE getNode()->ainfo << "async process has thrown -> finish processing" << std::endl;
                finishProcessing();
                throw;
            }
        }



    } catch(const std::exception& e) {
        setError(true, e.what());
    } catch(const Failure& f) {
        throw f;
    } catch(...) {
        throw Failure("Unknown exception caught in SubprocessNodeWorker.");
    }
    if(sync) {
        lock.unlock();
        //TRACE getNode()->ainfo << "sync process done -> finish processing" << std::endl;
        finishProcessing();
    }
}

void SubprocessNodeWorker::transmitParameter(const param::ParameterPtr &p)
{
    SubprocessChannel& channel = subprocess_.isChild() ? subprocess_.out : subprocess_.in;

    SerializationBuffer buffer = PacketSerializer::serializePacket(p);

    channel.write({SubprocessChannel::MessageType::PARAMETER_UPDATE, buffer.data(), buffer.size()});
}

void SubprocessNodeWorker::handleChangedParametersImpl(const Parameterizable::ChangedParameterList &changed_params)
{
    NodeWorker::handleChangedParametersImpl(changed_params);

    for(auto pair : changed_params) {
        if(param::ParameterPtr p = pair.first.lock()) {
            transmitParameter(p);
        }
    }
}
