/// HEADER
#include <csapex/utility/subprocess.h>

/// SYSTEM
#include <iostream>
#include <boost/optional.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <unistd.h>

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
      pid_(-1)
{
    active_ = true;
}

Subprocess::~Subprocess()
{
    if(isParent()) {
        bool is_shutdown = false;


        if(ctrl_out.hasMessage()) {
            SubprocessChannel::Message message = ctrl_out.read();
            if(message.type == SubprocessChannel::MessageType::CHILD_EXIT ||
                    message.type == SubprocessChannel::MessageType::CHILD_SIGNAL)
            {
                is_shutdown = true;
            }
        }

        if(!is_shutdown) {
            ctrl_in.write({SubprocessChannel::MessageType::SHUTDOWN, "shutdown"});
            SubprocessChannel::Message m = ctrl_out.read();
        }
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

pid_t Subprocess::fork(std::function<void()> child)
{
    pid_ = ::fork();
    if(pid_ == 0) {
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

        try {
            child();

        } catch(const std::exception& e) {
            if (active_) {
                out.write({SubprocessChannel::MessageType::CHILD_EXIT, e.what()});
            }

        } catch(...) {
            if (active_) {
                out.write({SubprocessChannel::MessageType::CHILD_EXIT, "unknown error"});
            }
        }

        subprocess_worker_.join();

        // then send the end of program signal
        ctrl_out.write({SubprocessChannel::MessageType::CHILD_EXIT, "eop"});

        std::quick_exit(0);
    }
    return pid_;
}

bool Subprocess::isActive() const
{
    return active_;
}
