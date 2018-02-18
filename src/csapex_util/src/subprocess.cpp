/// HEADER
#include <csapex/utility/subprocess.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <iostream>
#include <boost/optional.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <unistd.h>
#include <fstream>

using namespace csapex;

namespace detail
{
Subprocess* g_sp_instance = nullptr;

void sp_signal_handler(int signal)
{
    if(g_sp_instance) {
        g_sp_instance->handleSignal(signal);
    }
    std::quick_exit(0);
}
}

Subprocess::Subprocess(const std::string& name_space)
    : in(name_space + "_in", false, 65536),
      out(name_space + "_out", false, 65536),
      ctrl_in(name_space + "_ctrl", true, 1024),
      ctrl_out(name_space + "_ctrl", true, 1024),
      pid_(-1),
      is_shutdown(false),
      return_code(0)


{
    if (pipe(pipe_in)) {
        throw std::runtime_error("cannot create pipe for stdin");
    }
    if (pipe(pipe_out)) {
        close(pipe_in[0]);
        close(pipe_in[1]);
        throw std::runtime_error("cannot create pipe for stdout");
    }
    if (pipe(pipe_err)) {
        close(pipe_out[0]);
        close(pipe_out[1]);
        close(pipe_in[0]);
        close(pipe_in[1]);
        throw std::runtime_error("cannot create pipe for stderr");
    }

    active_ = true;
}

Subprocess::~Subprocess()
{
    if(isParent()) {
        if(!is_shutdown) {
            ctrl_in.write({SubprocessChannel::MessageType::SHUTDOWN, "shutdown"});
        }

        join();

        close(pipe_err[1]);
        close(pipe_out[0]);
        close(pipe_in[0]);
    }
}

bool Subprocess::isChild() const
{
    return pid_ == 0;
}

bool Subprocess::isParent() const
{
    return pid_ > 0;
}

void Subprocess::handleSignal(int signal)
{
    if(active_) {
        out.write({SubprocessChannel::MessageType::CHILD_SIGNAL, std::to_string(signal)});
        ctrl_out.write({SubprocessChannel::MessageType::CHILD_SIGNAL, std::to_string(signal)});
    }
}

pid_t Subprocess::fork(std::function<int()> child)
{
    pid_ = ::fork();
    if(pid_ == 0) {
        close(pipe_in[1]);
        close(pipe_out[0]);
        close(pipe_err[0]);

        dup2(pipe_in[0], 0);
        dup2(pipe_out[1], 1);
        dup2(pipe_err[1], 2);

        detail::g_sp_instance = this;
        std::signal(SIGABRT, detail::sp_signal_handler);
        std::signal(SIGFPE, detail::sp_signal_handler);
        std::signal(SIGILL, detail::sp_signal_handler);
        std::signal(SIGINT, detail::sp_signal_handler);
        std::signal(SIGSEGV, detail::sp_signal_handler);
        std::signal(SIGTERM, detail::sp_signal_handler);

        subprocess_worker_ = std::thread([this](){
            while(active_) {
                SubprocessChannel::Message m = ctrl_in.read();
                if(m.type == SubprocessChannel::MessageType::SHUTDOWN) {
                    active_ = false;
                    in.shutdown();
                    out.shutdown();
                }
            }
        });

        int return_code = 0;
        try {
            return_code = child();

        } catch(const std::exception& e) {
            if (active_) {
                out.write({SubprocessChannel::MessageType::CHILD_ERROR, e.what()});
            }
            return_code = -1;

        } catch(...) {
            if (active_) {
                out.write({SubprocessChannel::MessageType::CHILD_ERROR, "unknown error"});
            }

            return_code = -2;
        }

        active_ = false;
        ctrl_in.shutdown();
        subprocess_worker_.join();

        // then send the end of program signal
        ctrl_out.write({SubprocessChannel::MessageType::CHILD_EXIT, std::to_string(return_code)});

        std::quick_exit(0);

    } else {
        close(pipe_in[0]);
        close(pipe_out[1]);
        close(pipe_err[1]);

        parent_worker_ = std::thread([&]() {
            while(active_) {
                const std::size_t N = 1;
                char buf[N+1];
                std::size_t r;
                while((r = read(pipe_out[0], &buf, N)) > 0) {
                    buf[r] = 0;
                    child_cout << buf;
                }
                while((r = read(pipe_err[0], &buf, N)) > 0) {
                    buf[r] = 0;
                    child_cerr << buf;
                }
            }
        });
    }


    return pid_;
}

std::string Subprocess::getChildStdOut() const
{
    return child_cout.str();
}
std::string Subprocess::getChildStdErr() const
{
    return child_cerr.str();
}

void Subprocess::readCtrlOut()
{
    SubprocessChannel::Message message = ctrl_out.read();
    switch(message.type) {
    case SubprocessChannel::MessageType::CHILD_EXIT:
    case SubprocessChannel::MessageType::CHILD_SIGNAL:
    {
        is_shutdown = true;
        std::stringstream ss(message.toString());
        ss >> return_code;
    }
        break;
    case SubprocessChannel::MessageType::CHILD_ERROR:
        is_shutdown = true;
        return_code = 64;
        break;
    default:
        std::cout << "read an unknown message: " << message.toString() << std::endl;
        break;
    }
}

int Subprocess::join()
{
    apex_assert_hard(isParent());

    while(!is_shutdown) {
        readCtrlOut();
    }

    active_ = false;
    if(parent_worker_.joinable()) {
        parent_worker_.join();
    }

    return return_code;
}

bool Subprocess::isActive() const
{
    return active_;
}
