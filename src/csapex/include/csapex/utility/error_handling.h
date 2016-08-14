#ifndef ERROR_HANDLING_H
#define ERROR_HANDLING_H

/// PROJECT
#include <csapex/csapex_util_export.h>
#include <csapex/utility/slim_signal.hpp>

/// SYSTEM
#include <csignal>

namespace csapex
{
namespace error_handling
{

CSAPEX_UTILS_EXPORT void init();

CSAPEX_UTILS_EXPORT void siginthandler(int);
CSAPEX_UTILS_EXPORT void sigsegvhandler(int sig_num, siginfo_t * info, void * ucontext);
CSAPEX_UTILS_EXPORT void stop();
CSAPEX_UTILS_EXPORT void kill();

csapex::slim_signal::Signal<void()>& stop_request()
{
    static csapex::slim_signal::Signal<void()> s;
    return s;
}

}
}

#endif // ERROR_HANDLING_H
