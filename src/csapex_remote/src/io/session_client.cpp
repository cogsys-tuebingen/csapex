/// HEADER
#include <csapex/io/session_client.h>

/// SYSTEM
#include <iostream>

using namespace csapex;
using boost::asio::ip::tcp;

SessionClient::SessionClient(const std::string &ip, int port)
    : Session("client"),
      socket_impl(io_service),
      resolver(io_service),
      resolver_iterator(boost::asio::connect(socket_impl, resolver.resolve({ip, std::to_string(port)}))),
      io_service_running_(false)
{
    socket_.reset(new Socket(std::move(socket_impl)));
}

SessionClient::~SessionClient()
{
    socket_.reset();
}

void SessionClient::run()
{
    //io_service.run_one();
    io_service_running_ = true;
    io_service.run();

    stop();
}

void SessionClient::shutdown()
{
    io_service_running_ = false;
    io_service.stop();
    stop();
}

bool SessionClient::isRunning() const
{
    return Session::isRunning() && io_service_running_;
}
