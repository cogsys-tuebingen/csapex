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
      shm_block_(nullptr),
      pid_(-1)
{
}

void SubprocessNodeWorker::initialize()
{
    allocateSharedMemory();

    // ... generalize.......
    connection_types::GenericVectorMessage::registerType<connection_types::GenericValueMessage<int>>();
    connection_types::GenericVectorMessage::registerType<connection_types::GenericValueMessage<int>>(connection_types::type<connection_types::GenericValueMessage<int>>::name());

    pid_ = fork();

    if (pid_ == 0) {
        // NODE
        runSubprocessLoop();
    }

    NodeWorker::initialize();
}


void SubprocessNodeWorker::runSubprocessLoop()
{
    NodePtr node = getNode();

    try {
        observe(node->getParameterState()->parameter_changed, [&](param::Parameter* p){
            changed_parameters_.push_back(p);
        });

        scoped_lock<interprocess_mutex> lock(shm_block_->m);

        while(shm_block_->active){

            // wait for a message
            while(!shm_block_->has_message) {
                shm_block_->message_available.wait(lock);
                if(!shm_block_->active) {
                    std::quick_exit(0);
                }
            }

            shm_block_->has_message = false;

            switch(shm_block_->message_type)
            {
            case ShmBlock::MessageType::PARAMETER_UPDATE:
            {
                handleParameterUpdate();

            }
                break;

            case ShmBlock::MessageType::PROCESS:
            {
                handleProcessChild(lock);
            }
                break;
            }

            shm_block_->message_processed.notify_all();
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


void SubprocessNodeWorker::handleProcessChild(scoped_lock<interprocess_mutex>& lock)
{
    const ShmAllocator<int> alloc_inst (shm_segment->get_segment_manager());

    NodePtr node = getNode();

    YAML::Emitter result_emitter;

    try {
        std::pair<shared_string*, managed_shared_memory::size_type> input
                = shm_segment->find<shared_string> ("input_yaml");

        if(input.first) {
            YAML::Node yaml = YAML::Load(input.first->c_str());

            for(const YAML::Node& node : yaml) {
                UUID uuid = node["uuid"].as<UUID>();
                auto msg = MessageSerializer::deserializeMessage(node["data"]);

                InputPtr input = node_handle_->getInput(uuid);
                input->setToken(std::make_shared<Token>(msg));
            };
        }

        shm_segment->destroy<shared_string>("input_yaml");

        node->process(*node_handle_, *node);

        shm_block_->message_processed.notify_all();
        shm_block_->message_available.wait(lock);

        // send parameter updates
        for(param::Parameter* parameter : changed_parameters_) {
            transmitParameter(lock, getNode()->getParameter(parameter->name()));
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

    shm_block_->has_message = false;
    shm_block_->message_type = ShmBlock::MessageType::PROCESS;
    shm_segment->construct<shared_string>
            ("result_yaml")
            (result_emitter.c_str(), alloc_inst);
    shm_block_->message_available.notify_all();
}

void SubprocessNodeWorker::allocateSharedMemory()
{
    try {
        //        std::cout << "try to create shared memory object " << getUUID() << std::endl;
        shm_segment.reset(new managed_shared_memory(create_only, getUUID().getFullName().c_str(), 65536));

    } catch(const boost::interprocess::interprocess_exception& e) {
        //        std::cout << "could not create shared memory object: " << e.what() << std::endl;
        //        std::cout << "removing shared memory object " << getUUID() << std::endl;
        shared_memory_object::remove(getUUID().getFullName().c_str());
        shm_segment.reset(new managed_shared_memory(create_only, getUUID().getFullName().c_str(), 65536));
    }


    //Create a managed shared memory segment
    shm_block_ = shm_segment->construct<ShmBlock>("shm")();
}

SubprocessNodeWorker::~SubprocessNodeWorker()
{
    stopObserving();

    std::unique_lock<std::recursive_mutex> lock(sync);

    shm_block_->active = false;
    shm_block_->message_available.notify_all();

    using namespace boost::interprocess;
    shared_memory_object::remove(getUUID().getFullName().c_str());
}


void SubprocessNodeWorker::handleParameterUpdate()
{
    NodePtr node = getNode();

    std::pair<shared_buffer*, managed_shared_memory::size_type> input
            = shm_segment->find<shared_buffer> ("parameter");

    if(input.first) {
        SerializationBuffer buffer(input.first->data(), input.first->size());

        param::ParameterPtr p = std::dynamic_pointer_cast<param::Parameter>(PacketSerializer::deserializePacket(buffer));

        param::ParameterPtr existing = node->getParameter(p->name());
        existing->setValueFrom(*p);
    }

    shm_segment->destroy<shared_string>("parameter");

    shm_block_->message_available.notify_all();
}

bool SubprocessNodeWorker::handleProcessParent()
{
    std::pair<shared_string*, managed_shared_memory::size_type> result
            = shm_segment->find<shared_string> ("result_yaml");

    if(result.first) {
        YAML::Node yaml = YAML::Load(result.first->c_str());

        for(const YAML::Node& node : yaml) {
            UUID uuid = node["uuid"].as<UUID>();
            auto msg = MessageSerializer::deserializeMessage(node["data"]);

            OutputPtr output = node_handle_->getOutput(uuid);
            YAML::Emitter emitter;
            emitter << node["data"];
            msg::publish(output.get(), msg);
        };
    }

    //Deallocate previously allocated memory
    shm_segment->destroy<shared_string>("result_yaml");

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
            using namespace boost::interprocess;

            const ShmAllocator<int> alloc_inst (shm_segment->get_segment_manager());

            apex_assert_msg(pid_ != 0, "processNode called in subprocess");

            // APEX
            scoped_lock<interprocess_mutex> lock(shm_block_->m);

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

            shm_segment->construct<shared_string>
                    ("input_yaml")
                    (emitter.c_str(), alloc_inst);

            shm_block_->message_type = ShmBlock::MessageType::PROCESS;
            shm_block_->has_message = true;
            shm_block_->message_available.notify_all();

            shm_block_->message_processed.wait(lock);
            shm_block_->message_available.notify_all();

            bool done_processing = false;

            // wait for the end of processing

            while(!done_processing) {
                shm_block_->message_available.wait(lock);
                switch(shm_block_->message_type)
                {
                case ShmBlock::MessageType::PARAMETER_UPDATE:
                {
                    handleParameterUpdate();
                }
                    break;

                case ShmBlock::MessageType::PROCESS:
                {
                    done_processing = handleProcessParent();
                }
                    break;
                }

                shm_block_->message_processed.notify_all();
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

void SubprocessNodeWorker::transmitParameter(Lock &lock, const param::ParameterPtr &p)
{
    const ShmAllocator<int> alloc_inst (shm_segment->get_segment_manager());

    SerializationBuffer buffer = PacketSerializer::serializePacket(p);

    shm_segment->construct<shared_buffer>
            ("parameter")
            (buffer.data(), buffer.size(), alloc_inst);

    shm_block_->message_type = ShmBlock::MessageType::PARAMETER_UPDATE;
    shm_block_->has_message = true;
    shm_block_->message_available.notify_all();
    shm_block_->message_processed.wait(lock);
}

void SubprocessNodeWorker::handleChangedParametersImpl(const Parameterizable::ChangedParameterList &changed_params)
{
    NodeWorker::handleChangedParametersImpl(changed_params);

    using namespace boost::interprocess;
    apex_assert_hard(shm_block_);

    scoped_lock<interprocess_mutex> lock(shm_block_->m);

    //    apex_assert_msg(pid != 0, "handleChangedParametersImpl called in subprocess");
    // this method can be used in both directions!

    for(auto pair : changed_params) {
        if(param::ParameterPtr p = pair.first.lock()) {
            transmitParameter(lock, p);
        }
    }
}
