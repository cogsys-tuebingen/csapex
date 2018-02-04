#ifndef SUBPROCESS_NODE_WORKER_H
#define SUBPROCESS_NODE_WORKER_H

/// COMPONENT
#include <csapex/model/node_worker.h>

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/utility/utility_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/signal/signal_fwd.h>
#include <csapex/param/parameter.h>
#include <csapex/model/error_state.h>
#include <csapex/utility/uuid.h>
#include <csapex/model/notification.h>
#include <csapex/model/execution_mode.h>
#include <csapex/model/observer.h>
#include <csapex/model/notifier.h>
#include <csapex/model/activity_type.h>
#include <csapex/model/execution_state.h>
#include <csapex/model/activity_modifier.h>

/// SYSTEM
#include <map>
#include <functional>
#include <mutex>
#include <vector>
#include <atomic>
#include <csapex/utility/slim_signal.hpp>
#include <boost/optional.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>


namespace csapex {

class ProfilerImplementation;
class Interval;

template <typename T>
using ShmAllocator = boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager>;


class CSAPEX_EXPORT SubprocessNodeWorker : public NodeWorker
{
public:
    SubprocessNodeWorker(NodeHandlePtr node_handle);
    ~SubprocessNodeWorker();

    void initialize() override;
    void processNode() override;

protected:
    void handleChangedParametersImpl(const Parameterizable::ChangedParameterList& changed_params) override;

private:
    void allocateSharedMemory();
    void runSubprocessLoop();

private:
    std::shared_ptr<boost::interprocess::managed_shared_memory> shm_segment;

    using shared_string = typename boost::interprocess::basic_string<char, std::char_traits<char>, ShmAllocator<char>>;
    using shared_buffer = typename boost::interprocess::basic_string<uint8_t, std::char_traits<uint8_t>, ShmAllocator<uint8_t>>;

    struct ShmBlock
    {
        ShmBlock()
            : has_message(false),
              active(true)
        {
        }

        boost::interprocess::interprocess_mutex m;
        boost::interprocess::interprocess_condition message_available;
        boost::interprocess::interprocess_condition message_processed;

        enum class MessageType
        {
            PROCESS,
            PARAMETER_UPDATE
        };

        MessageType message_type;

        bool has_message;
        bool active;
    };

    ShmBlock* shm_block_;
    pid_t pid_;
};

}

#endif // SUBPROCESS_NODE_WORKER_H
