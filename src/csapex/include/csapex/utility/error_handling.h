#ifndef ERROR_HANDLING_H
#define ERROR_HANDLING_H

/// SYSTEM
#include <csignal>
#include <csapex/utility/slim_signal.hpp>


namespace csapex
{
namespace error_handling
{

void init();

void siginthandler(int);
void sigsegvhandler(int sig_num, siginfo_t * info, void * ucontext);
void stop();
void kill();

csapex::slim_signal::Signal<void()>& stop_request()
{
    static csapex::slim_signal::Signal<void()> s;
    return s;
}

}
}

#endif // ERROR_HANDLING_H
