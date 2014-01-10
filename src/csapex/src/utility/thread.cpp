/// HEADER
#include <csapex/utility/thread.h>

/// SYSTEM
#include <sys/prctl.h>

using namespace csapex;
using namespace thread;

void csapex::thread::set_name(const char* name)
{
    prctl(PR_SET_NAME,name,0,0,0);
}
