#ifndef SERVER_H
#define SERVER_H

/// PROJECT
#include <csapex/core/core_fwd.h>
#include <csapex/io/io_fwd.h>

/// SYSTEM
#include <boost/asio.hpp>
#include <thread>
#include <mutex>

namespace csapex
{

class Server
{
public:
    Server(CsApexCorePtr core, bool spin_thread = true);
    ~Server();

    void start();
    void stop();

private:
    void spin();
    void do_accept();

    boost::asio::io_service io_service_;
    boost::asio::ip::tcp::acceptor acceptor_;
    boost::asio::ip::tcp::socket socket_;

    CsApexCorePtr core_;

    std::atomic<bool> running_;

    bool spin_thread_;
    std::thread worker_thread_;

    std::recursive_mutex session_mutex_;
    std::vector<SessionPtr> sessions_;
};

}

#endif // SERVER_H
