/// HEADER
#include <csapex/utility/error_handling.h>

/// SYSTEM
#include <cstddef>
#include <cstdio>
#include <execinfo.h>
#include <stdio.h>
#include <stdlib.h>
#include <ucontext.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <cxxabi.h>
#include <vector>

// This structure mirrors the one found in /usr/include/asm/ucontext.h
typedef struct _sig_ucontext {
    unsigned long     uc_flags;
    struct ucontext   *uc_link;
    stack_t           uc_stack;
    struct sigcontext uc_mcontext;
    sigset_t          uc_sigmask;
} sig_ucontext_t;

void csapex::error_handling::init()
{
    signal(SIGINT, csapex::error_handling::siginthandler);

    struct sigaction sigact;
    memset (&sigact, '\0', sizeof(sigact));

    sigact.sa_sigaction = sigsegvhandler;
    sigact.sa_flags = SA_RESTART | SA_SIGINFO;

    std::vector<int> signal_vec;
    signal_vec.push_back(SIGSEGV);
    signal_vec.push_back(SIGFPE);
    signal_vec.push_back(SIGABRT);
//    signal_vec.push_back(SIGILL);

    for(unsigned i = 0; i < signal_vec.size(); ++i) {
        if (sigaction(signal_vec[i], &sigact, (struct sigaction *) NULL) != 0) {
            fprintf(stderr, "error setting signal handler for %d (%s)\n",
                    signal_vec[i], strsignal(signal_vec[i]));

            stop();
        }
    }
}

void csapex::error_handling::kill()
{
    raise(SIGKILL);
}

void csapex::error_handling::stop()
{
//    exit(EXIT_FAILURE);
    stop_request();
    // TODO: kill on timeout!
}

void csapex::error_handling::siginthandler(int)
{
    printf("User pressed Ctrl+C\n");
    stop_request();
}

void csapex::error_handling::sigsegvhandler(int sig_num, siginfo_t * info, void * ucontext)
{
    sig_ucontext_t * uc = (sig_ucontext_t *)ucontext;

#if defined(__i386__) // gcc specific
    void* caller_address = (void *) uc->uc_mcontext.eip; // EIP: x86 specific
#elif defined(__x86_64__) // gcc specific
    void* caller_address = (void *) uc->uc_mcontext.rip; // RIP: x86_64 specific
#else
#error Unsupported architecture. // TODO: Add support for other arch.
#endif

    std::cerr << "signal " << sig_num
              << " (" << strsignal(sig_num) << "), address is "
              << info->si_addr << " from " << caller_address
              << std::endl << std::endl;

    void * array[50];
    int size = backtrace(array, 50);

    array[1] = caller_address;

    char ** messages = backtrace_symbols(array, size);

    // skip first stack frame (points here)
    for (int i = 1; i < size && messages != NULL; ++i)
    {
        char *mangled_name = 0, *offset_begin = 0, *offset_end = 0;

        // find parantheses and +address offset surrounding mangled name
        for (char *p = messages[i]; *p; ++p)
        {
            if (*p == '(')
            {
                mangled_name = p;
            }
            else if (*p == '+')
            {
                offset_begin = p;
            }
            else if (*p == ')')
            {
                offset_end = p;
                break;
            }
        }

        // if the line could be processed, attempt to demangle the symbol
        if (mangled_name && offset_begin && offset_end &&
                mangled_name < offset_begin)
        {
            *mangled_name++ = '\0';
            *offset_begin++ = '\0';
            *offset_end++ = '\0';

            int status;
            char * real_name = abi::__cxa_demangle(mangled_name, 0, 0, &status);

            // if demangling is successful, output the demangled function name
            if (status == 0)
            {
                std::cerr << "[bt]: (" << i << ") " << messages[i] << " : "
                          << real_name << "+" << offset_begin << offset_end
                          << std::endl;

            }
            // otherwise, output the mangled function name
            else
            {
                std::cerr << "[bt]: (" << i << ") " << messages[i] << " : "
                          << mangled_name << "+" << offset_begin << offset_end
                          << std::endl;
            }
            free(real_name);
        }
        // otherwise, print the whole line
        else
        {
            std::cerr << "[bt]: (" << i << ") " << messages[i] << std::endl;
        }
    }
    std::cerr << std::endl;

    free(messages);

    raise(SIGKILL);
}
