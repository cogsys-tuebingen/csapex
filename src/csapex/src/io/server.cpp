/// HEADER
#include <csapex/io/server.h>

/// COMPONENT
#include <csapex/core/csapex_core.h>
#include <csapex/io/session.h>

/// SYSTEM
#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>

using namespace csapex;
using boost::asio::ip::tcp;

Server::Server(CsApexCorePtr core, bool spin_thread)
    : acceptor_(io_service_, tcp::endpoint(tcp::v4(), core->getSettings().get("port", 12345))),
      socket_(io_service_),
      core_(core),
      running_(false),
      spin_thread_(spin_thread)
{
}

Server::~Server()
{
    if(running_) {
        stop();
    }
    if(worker_thread_.joinable()) {
        worker_thread_.join();
    }
}

void Server::do_accept()
{
    acceptor_.async_accept(socket_,
                           [this](boost::system::error_code ec)
    {

        if (!ec)
        {
            SessionPtr session = std::make_shared<Session>(std::move(socket_), core_);

            SessionWeakPtr w_session = session;
            session->stopped.connect([this, w_session]() {
                std::unique_lock<std::recursive_mutex> lock(session_mutex_);
                auto pos = std::find(sessions_.begin(), sessions_.end(), w_session.lock());
                if(pos != sessions_.end()) {
                    sessions_.erase(pos);
                }
            });

            session->start();

            std::unique_lock<std::recursive_mutex> lock(session_mutex_);
            sessions_.push_back(session);
        }

        do_accept();
    });
}

void Server::spin()
{
    do_accept();

    while(running_) {
        try {
            io_service_.run();
        } catch(const std::exception& e) {
            std::cerr << "the tcp server has thrown an exception: " << e.what() << std::endl;
        }
    }
}

void Server::start()
{
    running_ = true;

    if(spin_thread_) {
        worker_thread_ = std::thread([this]() {
            spin();
        });
    } else {
        spin();
    }
}

void Server::stop()
{
    if(running_) {
        running_ = false;

        std::unique_lock<std::recursive_mutex> lock(session_mutex_);

        for(SessionPtr session : sessions_) {
            session->stop();
        }
        io_service_.stop();

        if(std::this_thread::get_id() != worker_thread_.get_id()) {
            if(spin_thread_ && worker_thread_.joinable()) {
                worker_thread_.join();
            }
        }
    }
}
