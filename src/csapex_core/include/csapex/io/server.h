#ifndef SERVER_H
#define SERVER_H

/// PROJECT
#include <csapex/model/observer.h>
#include <csapex/core/core_fwd.h>
#include <csapex/model/model_fwd.h>

namespace csapex
{
class Server : public Observer
{
protected:
    Server(CsApexCore& core);

public:
    ~Server();

    virtual void start() = 0;
    virtual void stop() = 0;

    virtual bool isRunning() const = 0;

protected:
    CsApexCore& core_;
};

}  // namespace csapex

#endif  // SERVER_H
