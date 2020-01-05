#ifndef SESSION_CLIENT_H
#define SESSION_CLIENT_H

/// PROJECT
#include <csapex/io/session.h>

/// SYSTEM
#include <boost/asio.hpp>
#include <boost/version.hpp>

namespace csapex
{
class SessionClient : public Session
{
public:
    SessionClient(const std::string& ip, int port);
    ~SessionClient() override;

    std::string getDescription() const override;

    void run() override;
    void shutdown() override;

    bool isRunning() const override;

private:
    boost::asio::io_service io_service;
    boost::asio::ip::tcp::socket socket_impl;
    boost::asio::ip::tcp::resolver resolver;
#if (BOOST_VERSION / 100000) >= 1 && (BOOST_VERSION / 100 % 1000) >= 66
    boost::asio::ip::tcp::resolver::endpoint_type resolver_iterator;
#else
    // deprecated in boost 1.66
    boost::asio::ip::tcp::resolver::iterator resolver_iterator;
#endif

    bool io_service_running_;

    std::string ip_;
    int port_;
};

}  // namespace csapex

#endif  // SESSION_CLIENT_H
