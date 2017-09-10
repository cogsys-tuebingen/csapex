#ifndef CHANNEL_H
#define CHANNEL_H

/// PROJECT
#include <csapex/utility/uuid.h>
#include <csapex/io/io_fwd.h>
#include <csapex/io/session.h>
#include <csapex/model/observer.h>

namespace csapex
{

namespace io
{

class Channel : public Observer
{
public:
    Channel(Session& session, const AUUID& name);

    template <typename Result, typename Request, typename Type, typename... Args>
    Result request(Type type, Args&&... args) const
    {
        auto res = session_.sendRequest<Request>(type, name_, std::forward<Args>(args)...);
        apex_assert_hard(res);
        return res->template getResult<Result>();
    }
    template <typename Request, typename Type, typename... Args>
    void sendRequest(Type type, Args&&... args) const
    {
        session_.sendRequest<Request>(type, name_, std::forward<Args>(args)...);
    }

public:
    slim_signal::Signal<void(const RawMessageConstPtr&)> raw_packet_received;

private:
    Session& session_;
    AUUID name_;
};

}

}

#endif // CHANNEL_H
