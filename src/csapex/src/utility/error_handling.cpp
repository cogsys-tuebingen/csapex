/// HEADER
#include <csapex/utility/error_handling.h>

/// SYSTEM
#include <cstddef>
#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <vector>

#ifdef WIN32
#else
#include <execinfo.h>
#include <ucontext.h>
#include <unistd.h>
#include <cxxabi.h>
#endif


#ifdef WIN32
#else
// This structure mirrors the one found in /usr/include/asm/ucontext.h
typedef struct _sig_ucontext {
    unsigned long     uc_flags;
    struct ucontext   *uc_link;
    stack_t           uc_stack;
    struct sigcontext uc_mcontext;
    sigset_t          uc_sigmask;
} sig_ucontext_t;
#endif


void csapex::error_handling::init()
{
    signal(SIGINT, csapex::error_handling::siginthandler);

#ifdef WIN32
	SetUnhandledExceptionFilter(csapex::error_handling::sigsegvhandler);
#else
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
        if (sigaction(signal_vec[i], &sigact, (struct sigaction *) nullptr) != 0) {
            fprintf(stderr, "error setting signal handler for %d (%s)\n",
                    signal_vec[i], strsignal(signal_vec[i]));

            stop();
        }
    }
#endif
}

void csapex::error_handling::kill()
{
#ifdef WIN32
#else
    raise(SIGKILL);
#endif
}

void csapex::error_handling::stop()
{
    stop_request()();
    // TODO: kill on timeout!
}

void csapex::error_handling::siginthandler(int)
{
    stop();
}

#ifdef WIN32
LONG WINAPI csapex::error_handling::sigsegvhandler(EXCEPTION_POINTERS * ExceptionInfo)
{
	switch (ExceptionInfo->ExceptionRecord->ExceptionCode)
	{
	case EXCEPTION_ACCESS_VIOLATION:
		fputs("Error: EXCEPTION_ACCESS_VIOLATION\n", stderr);
		break;
	case EXCEPTION_ARRAY_BOUNDS_EXCEEDED:
		fputs("Error: EXCEPTION_ARRAY_BOUNDS_EXCEEDED\n", stderr);
		break;
	case EXCEPTION_BREAKPOINT:
		fputs("Error: EXCEPTION_BREAKPOINT\n", stderr);
		break;
	case EXCEPTION_DATATYPE_MISALIGNMENT:
		fputs("Error: EXCEPTION_DATATYPE_MISALIGNMENT\n", stderr);
		break;
	case EXCEPTION_FLT_DENORMAL_OPERAND:
		fputs("Error: EXCEPTION_FLT_DENORMAL_OPERAND\n", stderr);
		break;
	case EXCEPTION_FLT_DIVIDE_BY_ZERO:
		fputs("Error: EXCEPTION_FLT_DIVIDE_BY_ZERO\n", stderr);
		break;
	case EXCEPTION_FLT_INEXACT_RESULT:
		fputs("Error: EXCEPTION_FLT_INEXACT_RESULT\n", stderr);
		break;
	case EXCEPTION_FLT_INVALID_OPERATION:
		fputs("Error: EXCEPTION_FLT_INVALID_OPERATION\n", stderr);
		break;
	case EXCEPTION_FLT_OVERFLOW:
		fputs("Error: EXCEPTION_FLT_OVERFLOW\n", stderr);
		break;
	case EXCEPTION_FLT_STACK_CHECK:
		fputs("Error: EXCEPTION_FLT_STACK_CHECK\n", stderr);
		break;
	case EXCEPTION_FLT_UNDERFLOW:
		fputs("Error: EXCEPTION_FLT_UNDERFLOW\n", stderr);
		break;
	case EXCEPTION_ILLEGAL_INSTRUCTION:
		fputs("Error: EXCEPTION_ILLEGAL_INSTRUCTION\n", stderr);
		break;
	case EXCEPTION_IN_PAGE_ERROR:
		fputs("Error: EXCEPTION_IN_PAGE_ERROR\n", stderr);
		break;
	case EXCEPTION_INT_DIVIDE_BY_ZERO:
		fputs("Error: EXCEPTION_INT_DIVIDE_BY_ZERO\n", stderr);
		break;
	case EXCEPTION_INT_OVERFLOW:
		fputs("Error: EXCEPTION_INT_OVERFLOW\n", stderr);
		break;
	case EXCEPTION_INVALID_DISPOSITION:
		fputs("Error: EXCEPTION_INVALID_DISPOSITION\n", stderr);
		break;
	case EXCEPTION_NONCONTINUABLE_EXCEPTION:
		fputs("Error: EXCEPTION_NONCONTINUABLE_EXCEPTION\n", stderr);
		break;
	case EXCEPTION_PRIV_INSTRUCTION:
		fputs("Error: EXCEPTION_PRIV_INSTRUCTION\n", stderr);
		break;
	case EXCEPTION_SINGLE_STEP:
		fputs("Error: EXCEPTION_SINGLE_STEP\n", stderr);
		break;
	case EXCEPTION_STACK_OVERFLOW:
		fputs("Error: EXCEPTION_STACK_OVERFLOW\n", stderr);
		break;
	default:
		fputs("Error: Unrecognized Exception\n", stderr);
		break;
	}
	fflush(stderr);
	/* If this is a stack overflow then we can't walk the stack, so just show
	where the error happened */
	if (EXCEPTION_STACK_OVERFLOW != ExceptionInfo->ExceptionRecord->ExceptionCode)
	{
		raise(SIGTERM);
	}
	else
	{
		raise(SIGTERM);
	}

	return EXCEPTION_EXECUTE_HANDLER;
}
#else
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
    for (int i = 1; i < size && messages != nullptr; ++i)
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
#endif