#ifndef ERROR_HANDLING_H
#define ERROR_HANDLING_H

/// SYSTEM
#include <csignal>

namespace csapex
{
namespace error_handling
{

void init();

void siginthandler(int);
void sigsegvhandler(int sig_num, siginfo_t * info, void * ucontext);

}
}

#endif // ERROR_HANDLING_H
