#ifndef ERROR_HANDLING_H
#define ERROR_HANDLING_H

/// SYSTEM
#include <csignal>
#include <boost/signals2.hpp>

namespace csapex
{
namespace error_handling
{

void init();

void siginthandler(int);
void sigsegvhandler(int sig_num, siginfo_t * info, void * ucontext);
void stop();
void kill();

boost::signals2::signal<void()> stop_request;

}
}

#endif // ERROR_HANDLING_H
