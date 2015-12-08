#ifndef BUFFER_H
#define BUFFER_H

/// PROJECT
#include <csapex/msg/message.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <mutex>

namespace csapex
{
class Buffer
{
public:
    Buffer();
    ~Buffer();

    void disable();
    void free();

    ConnectionType::ConstPtr read() const;

    void write(ConnectionType::ConstPtr message);

    bool isFilled() const;
    bool containsNoMessage() const;

private:
    bool enabled_;

    mutable std::mutex mutex_;
    ConnectionType::ConstPtr message_;
};
}

#endif // BUFFER_H
