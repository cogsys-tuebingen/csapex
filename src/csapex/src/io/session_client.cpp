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
      resolver_iterator(boost::asio::connect(socket_impl, resolver.resolve({ip, std::to_string(port)})))
{
    socket_.reset(new Socket(std::move(socket_impl)));
}

void SessionClient::run()
{
    io_service.run();
}

void SessionClient::shutdown()
{
    io_service.stop();
}
