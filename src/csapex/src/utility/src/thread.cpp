/// HEADER
#include <csapex/utility/thread.h>

/// SYSTEM
#ifdef WIN32
#else
#include <sys/prctl.h>
#endif

using namespace csapex;
using namespace thread;

namespace {
#ifdef WIN32
#else
    static __thread char thread_name_buffer[32] = { 0 };
#endif
}

void csapex::thread::set_name(const char* name)
{
#ifdef WIN32
#else
    prctl(PR_SET_NAME,name,0,0,0);
#endif
}

const char* csapex::thread::get_name()
{
#ifdef WIN32
	return "thread";
#else
    if (!thread_name_buffer[0])
        prctl(PR_GET_NAME,thread_name_buffer,0,0,0);
    return (const char *)thread_name_buffer;
#endif
}
