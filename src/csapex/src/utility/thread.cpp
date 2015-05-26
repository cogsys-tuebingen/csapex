/// HEADER
#include <csapex/utility/thread.h>

/// SYSTEM
#include <sys/prctl.h>

using namespace csapex;
using namespace thread;

namespace {
    static __thread char thread_name_buffer[32] = { 0 };
}

void csapex::thread::set_name(const char* name)
{
    prctl(PR_SET_NAME,name,0,0,0);
}

const char* csapex::thread::get_name()
{
    if (!thread_name_buffer[0])
        prctl(PR_GET_NAME,thread_name_buffer,0,0,0);
    return (const char *)thread_name_buffer;
}
