#ifndef SESSION_H
#define SESSION_H

/// PROJECT
#include <csapex/core/core_fwd.h>
#include <csapex/serialization/serialization_fwd.h>
#include <csapex/model/observer.h>
#include <csapex/io/io_fwd.h>
#include <csapex/utility/uuid.h>
#include <csapex/utility/slim_signal.hpp>

/// SYSTEM
#include <boost/asio.hpp>
#include <deque>
#include <future>
#include <unordered_map>
#include <boost/asio.hpp>

namespace csapex
{

class Session : public Observer, public std::enable_shared_from_this<Session>
{
public:
    using Socket = boost::asio::ip::tcp::socket;

    class NoConnectionException : public std::runtime_error
    {
    public:
        NoConnectionException()
            : std::runtime_error("No active connection.")
        {}
    };

public:
    Session(Socket socket, const std::string& name);
    ~Session();

    virtual void run();
    virtual void shutdown();

    virtual bool isRunning() const;

    void start();
    void stop();

    void stopForced();

    void write(const StreamableConstPtr &packet);
    void write(const std::string &message);

    //
    // REQUEST
    //
    ResponseConstPtr sendRequest(RequestConstPtr request);

    template <typename RequestWrapper>
    std::shared_ptr<typename RequestWrapper::ResponseT const>
    sendRequest(std::shared_ptr<typename RequestWrapper::RequestT const> request)
    {
        return std::dynamic_pointer_cast<typename RequestWrapper::RequestT const>(sendRequest(request));
    }

    template <typename RequestWrapper, typename... Args>
    std::shared_ptr<typename RequestWrapper::ResponseT const>
    sendRequest(Args&&... args)
    {
        auto res = sendRequest(std::make_shared<typename RequestWrapper::RequestT>(std::forward<Args>(args)...));
        apex_assert_hard(res);
        if(auto casted = std::dynamic_pointer_cast<typename RequestWrapper::ResponseT const>(res)) {
            return casted;
        } else {
            handleFeedback(res);
        }
        return nullptr;
    }


    //
    // NOTE
    //
    void sendNote(io::NoteConstPtr note);

    template <typename NoteWrapper, typename... Args>
    void sendNote(Args&&... args)
    {
        sendNote(std::make_shared<NoteWrapper>(std::forward<Args>(args)...));
    }

    //
    // BROADCAST
    //
    template <typename BroadcastWrapper, typename... Args>
    void sendBroadcast(Args&&... args)
    {
        write(std::make_shared<BroadcastWrapper>(std::forward<Args>(args)...));
    }

    //
    // CHANNEL
    //
    io::ChannelPtr openChannel(const AUUID& name);

protected:
    Session(const std::string& name);

    void handleFeedback(const ResponseConstPtr& fb);

public:
    slim_signal::Signal<void(Session*)> started;
    slim_signal::Signal<void(Session*)> stopped;

    slim_signal::Signal<void(const StreamableConstPtr&)> packet_received;
    slim_signal::Signal<void(const BroadcastMessageConstPtr&)> broadcast_received;

    slim_signal::Signal<void(const RawMessageConstPtr&)> &raw_packet_received(const AUUID& uuid);

protected:
    void mainLoop();

    void read_async();

    void write_packet(SerializationBuffer &buffer);

protected:
    std::thread packet_handler_thread_;
    std::unique_ptr<Socket> socket_;

    uint8_t next_request_id_;


    std::recursive_mutex packets_mutex_;
    std::condition_variable_any packets_available_;
    std::deque<StreamableConstPtr> packets_received_;
    std::deque<StreamableConstPtr> packets_to_send_;

    std::recursive_mutex open_requests_mutex_;
    std::map<uint8_t, std::promise<ResponseConstPtr>*> open_requests_;

    std::recursive_mutex running_mutex_;
    std::atomic<bool> running_;
    std::atomic<bool> is_live_;
    std::atomic<bool> was_live_;

    // TODO: use the more general channel
    std::unordered_map<AUUID, std::shared_ptr<slim_signal::Signal<void(const RawMessageConstPtr&)>>, AUUID::Hasher> auuid_to_signal_;

    std::unordered_map<AUUID, io::ChannelPtr, AUUID::Hasher> channels_;

private:
    std::string name_;
    bool is_valid_;
};

}

#endif // SESSION_H
