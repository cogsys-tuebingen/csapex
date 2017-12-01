#ifndef PROXY_H
#define PROXY_H

/// PROJECT
#include <csapex/io/io_fwd.h>
#include <csapex/io/session.h>
#include <csapex/utility/assert.h>

namespace csapex
{

class Proxy
{
public:
    Proxy(const SessionPtr &session);
    virtual ~Proxy();

protected:
    virtual void handleBroadcast(const BroadcastMessageConstPtr& message);

protected:
    template <typename Result, typename Request, typename Type, typename... Args>
    Result request(Type type, Args&&... args) const
    {
        auto res = session_->sendRequest<Request>(type, std::forward<Args>(args)...);
        apex_assert_hard(res);
        return res->template getResult<Result>();
    }

protected:
    SessionPtr session_;

    slim_signal::ScopedConnection broadcast_connection_;
};

}

#endif // PROXY_H
