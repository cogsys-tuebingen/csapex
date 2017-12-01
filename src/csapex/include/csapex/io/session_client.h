#ifndef SESSION_CLIENT_H
#define SESSION_CLIENT_H

/// PROJECT
#include <csapex/io/session.h>

/// SYSTEM
#include <boost/asio.hpp>

namespace csapex
{
class SessionClient : public Session
{
public:
    SessionClient(const std::string &ip, int port);
    ~SessionClient();

    void run() override;
    void shutdown() override;

    bool isRunning() const override;

private:
    boost::asio::io_service io_service;
    boost::asio::ip::tcp::socket socket_impl;
    boost::asio::ip::tcp::resolver resolver;
    boost::asio::ip::tcp::resolver::iterator resolver_iterator;

    bool io_service_running_;
};

}

#endif // SESSION_CLIENT_H
