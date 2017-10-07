/// HEADER
#include <csapex/io/server.h>

/// COMPONENT
#include <csapex/core/csapex_core.h>
#include <csapex/io/session.h>
#include <csapex/io/protcol/notification_message.h>
#include <csapex/io/request.h>
#include <csapex/io/response.h>
#include <csapex/io/feedback.h>
#include <csapex/command/update_parameter.h>
#include <csapex/model/graph_facade_local.h>
#include <csapex/model/node_facade_local.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/io/protcol/parameter_changed.h>
#include <csapex/io/protcol/command_broadcasts.h>
#include <csapex/io/protcol/graph_broadcasts.h>
#include <csapex/io/graph_server.h>

/// SYSTEM
#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>

using namespace csapex;
using boost::asio::ip::tcp;

Server::Server(CsApexCorePtr core, bool spin_thread)
    : acceptor_(io_service_, tcp::endpoint(tcp::v4(), core->getSettings().getTemporary("port", 12345))),
      socket_(io_service_),
      core_(core),
      running_(false),
      spin_thread_(spin_thread)
{
    observe(core_->shutdown_requested, [this](){
        stop();
    });
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
            SessionPtr session = std::make_shared<Session>(std::move(socket_));

            SessionWeakPtr w_session = session;
            session->stopped.connect([this, session]() {
                std::unique_lock<std::recursive_mutex> lock(session_mutex_);
                auto pos = std::find(sessions_.begin(), sessions_.end(), session);
                if(pos != sessions_.end()) {
                    sessions_.erase(pos);
                }
            });

            observe(session->packet_received, [this, w_session](const StreamableConstPtr& packet){
                if(SessionPtr session = w_session.lock()) {
                    handlePacket(session, packet);
                }
            });

            // settings
            observe(core_->getSettings().setting_changed, [this, w_session](const std::string& name) {
                if(SessionPtr session = w_session.lock()) {
                    UUID id = UUIDProvider::makeUUID_without_parent(std::string(":") + name);
                    param::ParameterPtr p = core_->getSettings().get(name);

                    boost::any any;
                    p->get_unsafe(any);
                    session->write(std::make_shared<ParameterChanged>(id, any));
                }
            });

            // core
            observe(core_->getCommandDispatcher()->state_changed, [this, w_session]() {
                if(SessionPtr session = w_session.lock()) {
                    session->sendBroadcast<CommandBroadcasts>(CommandBroadcasts::CommandBroadcastType::StateChanged);
                }
            });
            observe(core_->getCommandDispatcher()->dirty_changed, [this, w_session](bool dirty) {
                if(SessionPtr session = w_session.lock()) {
                    session->sendBroadcast<CommandBroadcasts>(CommandBroadcasts::CommandBroadcastType::DirtyChanged, dirty);
                }
            });

            // graphs and nodes
            GraphServerPtr graph_server = std::make_shared<GraphServer>(session);
            graph_servers_[session.get()] = graph_server;
            graph_server->startObserving(core_->getRoot());

            session->start();

            std::unique_lock<std::recursive_mutex> lock(session_mutex_);
            sessions_.push_back(session);
        }

        do_accept();
    });
}
void Server::handlePacket(const SessionPtr& session, const StreamableConstPtr& packet)
{
    if(CommandConstPtr cmd = std::dynamic_pointer_cast<Command const>(packet)) {
        if(!cmd) {
            session->write("unknown command received");
        } else {
            session->write(cmd->getDescription());
            core_->getCommandDispatcher()->execute(cmd->clone<Command>());
        }

    } else if(RequestConstPtr request = std::dynamic_pointer_cast<Request const>(packet)) {
        StreamableConstPtr response;
        try {
            response = request->execute(session, *core_);

        } catch(const std::exception& e) {
            response = std::make_shared<Feedback>(std::string("Request has thrown an exception: ") + e.what(), request->getRequestID());

        } catch(...) {
            response = std::make_shared<Feedback>(std::string("Request has failed with unkown cause."), request->getRequestID());
        }

        if(!response) {
            response = std::make_shared<Feedback>(std::string("Request failed to produce a response"), request->getRequestID());
        }

        session->write(response);

    } else {
        session->write("packet with unknown type received");
    }
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

        {
            std::unique_lock<std::recursive_mutex> lock(session_mutex_);

            for(SessionPtr session : sessions_) {
                session->stop();
            }
            sessions_.clear();
        }

        io_service_.stop();

        if(std::this_thread::get_id() != worker_thread_.get_id()) {
            if(spin_thread_ && worker_thread_.joinable()) {
                try {
                    worker_thread_.join();
                } catch(const std::runtime_error& e) {
                    std::cerr << "failed to join server thread: " << e.what() << std::endl;
                }
            }
        }
    }
}
