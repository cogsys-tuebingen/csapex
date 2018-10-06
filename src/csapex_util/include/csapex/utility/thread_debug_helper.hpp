#ifndef THREAD_DEBUG_HELPER_HPP
#define THREAD_DEBUG_HELPER_HPP

/// PROJECT
#include <csapex/utility/assert.h>

/// SYSTEM
#include <thread>

namespace csapex
{

class ThreadDebugHelper
{
public:
    virtual ~ThreadDebugHelper() {

    }

protected:
    void assertSameThreadId()
    {
        apex_assert_equal_hard(processing_thread_id_, std::this_thread::get_id());
    }
    void rememberThreadId()
    {
        processing_thread_id_ = std::this_thread::get_id();
    }

    bool hasThreadId() const
    {
        return processing_thread_id_ != std::thread::id();
    }

private:
    std::thread::id processing_thread_id_;
};

}

#endif // THREAD_DEBUG_HELPER_HPP
