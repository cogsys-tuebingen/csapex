#ifndef SERVER_H
#define SERVER_H

/// PROJECT
#include <csapex/model/observer.h>
#include <csapex/core/core_fwd.h>
#include <csapex/io/io_fwd.h>
#include <csapex/model/model_fwd.h>
#include <csapex/serialization/serialization_fwd.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <boost/asio.hpp>
#include <thread>
#include <mutex>
#include <map>

namespace csapex
{

class Server : public Observer
{
public:
    Server(CsApexCorePtr core, bool spin_thread = true);
    ~Server();

    void start();
    void stop();

private:
    void spin();
    void do_accept();

    void handlePacket(const SessionPtr &session, const StreamableConstPtr &packet);

    boost::asio::io_service io_service_;
    boost::asio::ip::tcp::acceptor acceptor_;
    boost::asio::ip::tcp::socket socket_;

    CsApexCorePtr core_;

    std::atomic<bool> running_;

    bool spin_thread_;
    std::thread worker_thread_;

    std::recursive_mutex session_mutex_;
    std::vector<SessionPtr> sessions_;

    std::map<Session*, GraphServerPtr> graph_servers_;
};

}

#endif // SERVER_H
