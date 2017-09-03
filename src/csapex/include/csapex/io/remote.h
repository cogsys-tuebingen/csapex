#ifndef REMOTE_H
#define REMOTE_H

/// PROJECT
#include <csapex/io/io_fwd.h>
#include <csapex/io/session.h>
#include <csapex/utility/assert.h>

namespace csapex
{

class Remote
{
public:
    Remote(SessionPtr session);
    virtual ~Remote();

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

#endif // REMOTE_H
