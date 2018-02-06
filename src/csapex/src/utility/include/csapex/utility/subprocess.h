#ifndef SUBPROCESS_H
#define SUBPROCESS_H

/// COMPONENT
#include <csapex/utility/subprocess_channel.h>

#include <iostream>
#include <csignal>
#include <thread>

namespace csapex
{

class Subprocess
{
public:
    Subprocess(const std::string& name_space);
    ~Subprocess();

    void handleSignal(int signal);
    pid_t fork(std::function<void()> child);

    bool isActive() const;

    bool isParent() const;
    bool isChild() const;

public:
    SubprocessChannel in;
    SubprocessChannel out;

private:
    SubprocessChannel ctrl_in;
    SubprocessChannel ctrl_out;

private:
    pid_t pid_;
    bool active_;

    std::thread subprocess_worker_;
};


}

#endif // SUBPROCESS_H
