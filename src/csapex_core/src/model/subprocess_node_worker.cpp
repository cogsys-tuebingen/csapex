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
#include <csapex/serialization/io/std_io.h>
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
#include <future>
#include <csignal>
#include <boost/interprocess/exceptions.hpp>

using namespace csapex;

SubprocessNodeWorker::SubprocessNodeWorker(NodeHandlePtr node_handle)
    : NodeWorker(node_handle),
      pid_(-1),
      subprocess_(new Subprocess(node_handle->getUUID().getFullName()))
{
}

void SubprocessNodeWorker::initialize()
{

    pid_ = subprocess_->fork([this](){
        runSubprocessLoop();
    });

    observe(node_handle_->connector_created, [this](ConnectablePtr c, bool internal){
        if(!internal) {
            node_handle_->execution_requested([this, c](){
                SerializationBuffer msg;
                c->getDescription().serialize(msg);
                subprocess_->in.write({SubprocessChannel::MessageType::PORT_ADD, msg.data(), msg.size()});
            });
        }
    });
    observe(node_handle_->node_state_changed, [this](){
        node_handle_->execution_requested([this](){
            SerializationBuffer msg;
            getNodeHandle()->getNodeState()->serialize(msg);
            subprocess_->in.write({SubprocessChannel::MessageType::NODE_STATE_CHANGED, msg.data(), msg.size()});
        });
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

        while(subprocess_->isActive()) {

            // wait for a message
            SubprocessChannel::Message msg = subprocess_->in.read();
            if(msg.type == SubprocessChannel::MessageType::SHUTDOWN) {
                return;
            }

            switch(msg.type)
            {
            case SubprocessChannel::MessageType::PARAMETER_UPDATE:
                handleParameterUpdate(msg);
                break;

            case SubprocessChannel::MessageType::PROCESS_SYNC:
            case SubprocessChannel::MessageType::PROCESS_ASYNC:
                handleProcessChild(msg);
                break;


            case SubprocessChannel::MessageType::PROCESS_SLOT:
                handleProcessSlotChild(msg);
                break;

            case SubprocessChannel::MessageType::NODE_STATE_CHANGED:
                node->stateChanged();
                break;

            case SubprocessChannel::MessageType::PORT_ADD:
            {
                ConnectorDescription des;
                SerializationBuffer buffer(msg.data, msg.length);
                des.deserialize(buffer);
                if(des.is_variadic) {
                    switch(des.connector_type) {
                    case ConnectorType::INPUT:
                    {
                        auto vi = std::dynamic_pointer_cast<VariadicInputs>(getNode());
                        vi->createVariadicInput(des.token_type, des.label, des.optional);
                    }
                        break;
                    case ConnectorType::OUTPUT:
                    {
                        auto vo = std::dynamic_pointer_cast<VariadicOutputs>(getNode());
                        vo->createVariadicOutput(des.token_type, des.label);
                    }
                        break;
                    default:
                        break;
                    }
                } else {
                    switch(des.connector_type) {
                    case ConnectorType::INPUT:
                        getNodeHandle()->addInput(des.token_type, des.label, des.optional);
                        break;
                    case ConnectorType::OUTPUT:
                        getNodeHandle()->addOutput(des.token_type, des.label);
                        break;
                    default:
                        break;
                    }
                }

            }
                break;

            default:
                node->aerr << "subprocess received unknown message: " << (int) msg.type << std::endl;
                break;
            }
        }


    } catch(const SubprocessChannel::ShutdownException& e) {
        // ignore

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
    handleChangedParameters();

    NodePtr node = getNode();

    apex_assert_hard(node->canRunInSeparateProcess());

    try {
        if(msg.data) {
            YAML::Node yaml = YAML::Load(msg.toString());
            for(const YAML::Node& node : yaml) {
                UUID uuid = node["uuid"].as<UUID>();
                auto msg = MessageSerializer::deserializeMessage(node["data"]);

                InputPtr input = node_handle_->getInput(uuid);
                apex_assert_hard_msg(input, std::string("could not get input ") + uuid.getFullName());

                input->setToken(std::make_shared<Token>(msg));
            };
        }

        if(msg.type == SubprocessChannel::MessageType::PROCESS_SYNC) {
            node->process(*node_handle_, *node);
            finishHandleProcessChild();

        } else if(msg.type == SubprocessChannel::MessageType::PROCESS_ASYNC) {
            node->process(*node_handle_, *node, [this, node](ProcessingFunction f) {
                finishHandleProcessChild();
            });
        }

    } catch(const std::exception& e) {
        node->aerr << "subprocess: " << e.what() << std::endl;
        finishHandleProcessChild();
    } catch(const Failure& f) {
        node->aerr << "subprocess failure: " << f.what() << std::endl;
        finishHandleProcessChild();
    } catch(...) {
        node->aerr << "unknown error in subprocess" << std::endl;
        finishHandleProcessChild();
    }
}


void SubprocessNodeWorker::handleProcessSlotChild(const SubprocessChannel::Message& msg)
{
    handleChangedParameters();

    NodePtr node = getNode();
    apex_assert_hard(node->canRunInSeparateProcess());

    try {
        if(msg.data) {
            YAML::Node yaml = YAML::Load(msg.toString());
            UUID uuid = yaml["uuid"].as<UUID>();
            auto msg = MessageSerializer::deserializeMessage(yaml["data"]);

            SlotPtr slot = node_handle_->getSlot(uuid);
            apex_assert_hard_msg(slot, std::string("could not get slot ") + uuid.getFullName());

            slot->setToken(std::make_shared<Token>(msg));
            slot->handleEvent();
        }

    } catch(const std::exception& e) {
        node->aerr << "handleProcessSlotChild: " << e.what() << std::endl;
    } catch(const Failure& f) {
        node->aerr << "handleProcessSlotChild failure: " << f.what() << std::endl;
    } catch(...) {
        node->aerr << "unknown error in handleProcessSlotChild" << std::endl;
    }
    finishHandleProcessChild();
}

void SubprocessNodeWorker::finishHandleProcessChild()
{
    NodePtr node = getNode();

    YAML::Emitter result_emitter;
    try {
        // send parameter updates
        for(param::Parameter* parameter : changed_parameters_) {
            transmitParameter(getNode()->getParameter(parameter->name()));
        }

        changed_parameters_.clear();


        // send result
        YAML::Node yaml(YAML::NodeType::Sequence);

        for(const OutputPtr& output : node_handle_->getExternalOutputs()) {
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

        for(const EventPtr& event: node_handle_->getExternalEvents()) {
            auto msg = event->getAddedToken();

            if(msg) {
                YAML::Node node(YAML::NodeType::Map);
                node["uuid"] = event->getUUID();
                // TODO serialize token! (+ activity, ...)
                node["data"] = MessageSerializer::serializeMessage(*msg->getTokenData());

                YAML::Emitter emitter;
                emitter << node;

                yaml.push_back(node);
            }
        }

        result_emitter << yaml;

    } catch(const std::exception& e) {
        node->aerr << "finishHandleProcessChild: " << e.what() << std::endl;
    } catch(const Failure& f) {
        node->aerr << "finishHandleProcessChild failure: " << f.what() << std::endl;
    } catch(...) {
        node->aerr << "unknown error in finishHandleProcessChild" << std::endl;
    }

    subprocess_->flush();

    subprocess_->out.write({SubprocessChannel::MessageType::PROCESS_FINISHED, result_emitter.c_str()});
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
        existing->cloneDataFrom(*p);
    }
}

void SubprocessNodeWorker::handleProcessParent(const SubprocessChannel::Message& msg)
{
    if(msg.data) {
        YAML::Node yaml = YAML::Load(msg.toString());

        for(const YAML::Node& node : yaml) {
            UUID uuid = node["uuid"].as<UUID>();
            auto msg = MessageSerializer::deserializeMessage(node["data"]);

            ConnectorPtr connector = node_handle_->getConnector(uuid);
            if(OutputPtr output = std::dynamic_pointer_cast<Output>(connector)) {
                msg::publish(output.get(), msg);

            } else if(EventPtr event = std::dynamic_pointer_cast<Event>(connector)){
                TokenPtr token = std::make_shared<Token>(msg);
                event->triggerWith(token);
            }
        };
    }
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
            startSubprocess(SubprocessChannel::MessageType::PROCESS_SYNC);

        } else {
            async_future_ = std::async(std::launch::async, [this](){
                startSubprocess(SubprocessChannel::MessageType::PROCESS_ASYNC);
                finishSubprocess();
                finishProcessing();
            });
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

        finishSubprocess();
        finishProcessing();
    }
}

void SubprocessNodeWorker::startSubprocess(const SubprocessChannel::MessageType type)
{
    YAML::Node yaml(YAML::NodeType::Sequence);

    for(const InputPtr& input : node_handle_->getExternalInputs()) {
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

    subprocess_->in.write({type, emitter.c_str()});
}

void SubprocessNodeWorker::processSlot(const SlotWeakPtr &slot_w)
{
    apex_assert_msg(pid_ != 0, "processSlot called in subprocess");

    SlotPtr slot = slot_w.lock();
    apex_assert_hard(slot);

    try {
        startSubprocessSlot(slot);

    } catch(const std::exception& e) {
        setError(true, e.what());
    } catch(const Failure& f) {
        throw f;
    } catch(...) {
        throw Failure("Unknown exception caught in SubprocessNodeWorker.");
    }
}

void SubprocessNodeWorker::startSubprocessSlot(const SlotPtr& slot)
{
    TokenPtr token = slot->getToken();
    apex_assert_hard(token);

    auto msg = msg::getMessage(slot.get());

    if(msg) {
        YAML::Node yaml(YAML::NodeType::Map);
        yaml["uuid"] = slot->getUUID();
        // TODO serialize token! (+ activity, ...)
        yaml["data"] = MessageSerializer::serializeMessage(*msg);

        YAML::Emitter emitter;
        emitter << yaml;

        subprocess_->in.write({SubprocessChannel::MessageType::PROCESS_SLOT, emitter.c_str()});

        finishSubprocess();
    }
}

void SubprocessNodeWorker::finishSubprocess()
{
    bool done_processing = false;
    bool crashed = false;

    // wait for the end of processing

    while(!done_processing) {
        SubprocessChannel::Message msg = subprocess_->out.read();
        switch(msg.type)
        {
        case SubprocessChannel::MessageType::PARAMETER_UPDATE:
            handleParameterUpdate(msg);
            break;

        case SubprocessChannel::MessageType::PROCESS_FINISHED:
            handleProcessParent(msg);
            done_processing = true;
            break;

        case SubprocessChannel::MessageType::CHILD_SIGNAL:
            setError(true, std::string("Child has raised signal: ") + strsignal(atoi(msg.toString().c_str())), ErrorLevel::ERROR);
            crashed = true;
            done_processing = true;
            break;

        case SubprocessChannel::MessageType::CHILD_EXIT:
            setError(true, "Child unexpectedly quit", ErrorLevel::ERROR);
            crashed = true;
            done_processing = true;
            break;

        default:
            setError(true, std::string("Unhandled subprocess message: ") + std::to_string((int) msg.type ), ErrorLevel::WARNING);
            break;
        }
    }

    if(crashed) {
        getNode()->aerr << "*** node crashed! ***" << std::endl;

        std::string out = subprocess_->getChildStdOut();
        if(!out.empty()) {
            getNode()->aerr << "*** STDOUT: ***" << std::endl;
            getNode()->aerr << out << std::endl;
        }

        std::string err = subprocess_->getChildStdErr();
        if(!err.empty()) {
            getNode()->aerr << "*** STDERR: ***" << std::endl;
            getNode()->aerr << err << std::endl;
        }

        getNode()->aerr << "*** restarting subprocess ***" << std::endl;

        subprocess_.reset(new Subprocess(getUUID().getFullName()));
        pid_ = subprocess_->fork([this](){
            runSubprocessLoop();
        });

    } else {
        std::string out = subprocess_->getChildStdOut();
        if(!out.empty()) {
            getNode()->ainfo << out << std::endl;
        }

        std::string err = subprocess_->getChildStdErr();
        if(!err.empty()) {
            getNode()->aerr << err << std::endl;
        }
    }
}

void SubprocessNodeWorker::finishProcessing()
{
    //    finishSubprocess();

    NodeWorker::finishProcessing();
}

void SubprocessNodeWorker::transmitParameter(const param::ParameterPtr &p)
{
    SubprocessChannel& channel = subprocess_->isChild() ? subprocess_->out : subprocess_->in;

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
