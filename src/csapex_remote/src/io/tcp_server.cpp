/// HEADER
#include <csapex/io/tcp_server.h>

/// COMPONENT
#include <csapex/core/csapex_core.h>
#include <csapex/io/session.h>
#include <csapex/io/protcol/notification_message.h>
#include <csapex/io/request.h>
#include <csapex/io/response.h>
#include <csapex/io/feedback.h>
#include <csapex/command/update_parameter.h>
#include <csapex/model/graph_facade_impl.h>
#include <csapex/model/node_facade_impl.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/io/protcol/parameter_changed.h>
#include <csapex/io/protcol/command_broadcasts.h>
#include <csapex/io/protcol/graph_broadcasts.h>
#include <csapex/io/protcol/core_notes.h>
#include <csapex/io/graph_server.h>

/// SYSTEM
#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>

using namespace csapex;
using boost::asio::ip::tcp;

TcpServer::TcpServer(CsApexCore& core, bool spin_thread)
  : Server(core), acceptor_(io_service_, tcp::endpoint(tcp::v4(), core.getSettings().get<int>("port"))), socket_(io_service_), running_(false), spin_thread_(spin_thread)
{
}

TcpServer::~TcpServer()
{
    if (running_) {
        stop();
    }
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
}

void TcpServer::do_accept()
{
    acceptor_.async_accept(socket_, [this](boost::system::error_code ec) {
        if (!ec) {
            SessionPtr session = std::make_shared<Session>(std::move(socket_), "server");

            startSession(session);
        }

        do_accept();
    });
}

void TcpServer::startSession(SessionPtr session)
{
    SessionWeakPtr w_session = session;
    observe(session->stopped, [this, w_session](Session* session) { stopSession(session); });

    observe(session->packet_received, [this, w_session](const StreamableConstPtr& packet) {
        if (SessionPtr session = w_session.lock()) {
            handlePacket(session, packet);
        }
    });

    // settings
    observe(core_.getSettings().setting_changed, [this, w_session](const std::string& name) {
        if (SessionPtr session = w_session.lock()) {
            UUID id = UUIDProvider::makeUUID_without_parent(std::string(":") + name);
            param::ParameterPtr p = core_.getSettings().get(name);

            boost::any any;
            p->get_unsafe(any);
            session->write(std::make_shared<ParameterChanged>(id, any));
        }
    });

    // core
    observe(core_.getCommandDispatcher()->state_changed, [w_session]() {
        if (SessionPtr session = w_session.lock()) {
            session->sendBroadcast<CommandBroadcasts>(CommandBroadcasts::CommandBroadcastType::StateChanged);
        }
    });
    observe(core_.getCommandDispatcher()->dirty_changed, [w_session](bool dirty) {
        if (SessionPtr session = w_session.lock()) {
            session->sendBroadcast<CommandBroadcasts>(CommandBroadcasts::CommandBroadcastType::DirtyChanged, dirty);
        }
    });
    observe(core_.getCommandDispatcher()->dirty_changed, [w_session](bool dirty) {
        if (SessionPtr session = w_session.lock()) {
            session->sendBroadcast<CommandBroadcasts>(CommandBroadcasts::CommandBroadcastType::DirtyChanged, dirty);
        }
    });

    observe(core_.config_changed, [w_session]() {
        if (SessionPtr session = w_session.lock()) {
            session->sendNote<CoreNote>(CoreNoteType::ConfigChanged);
        }
    });
    observe(core_.begin_step, [w_session]() {
        if (SessionPtr session = w_session.lock()) {
            session->sendNote<CoreNote>(CoreNoteType::StepBegin);
        }
    });
    observe(core_.end_step, [w_session]() {
        if (SessionPtr session = w_session.lock()) {
            session->sendNote<CoreNote>(CoreNoteType::StepEnd);
        }
    });

    observe(core_.status_changed, [w_session](const std::string& msg) {
        if (SessionPtr session = w_session.lock()) {
            session->sendNote<CoreNote>(CoreNoteType::StatusChanged, msg);
        }
    });

    observe(core_.new_node_type, [w_session]() {
        if (SessionPtr session = w_session.lock()) {
            session->sendNote<CoreNote>(CoreNoteType::NewNodeType);
        }
    });
    observe(core_.new_snippet_type, [w_session]() {
        if (SessionPtr session = w_session.lock()) {
            session->sendNote<CoreNote>(CoreNoteType::NewSnippetType);
        }
    });
    observe(core_.reset_requested, [w_session]() {
        if (SessionPtr session = w_session.lock()) {
            session->sendNote<CoreNote>(CoreNoteType::ResetRequested);
        }
    });
    observe(core_.reset_done, [w_session]() {
        if (SessionPtr session = w_session.lock()) {
            session->sendNote<CoreNote>(CoreNoteType::ResetDone);
        }
    });
    observe(core_.saved, [w_session]() {
        if (SessionPtr session = w_session.lock()) {
            session->sendNote<CoreNote>(CoreNoteType::Saved);
        }
    });
    observe(core_.loaded, [w_session]() {
        if (SessionPtr session = w_session.lock()) {
            session->sendNote<CoreNote>(CoreNoteType::Loaded);
        }
    });
    observe(core_.paused, [w_session](bool paused) {
        if (SessionPtr session = w_session.lock()) {
            session->sendNote<CoreNote>(CoreNoteType::Paused, paused);
        }
    });
    observe(core_.stepping_enabled, [w_session]() {
        if (SessionPtr session = w_session.lock()) {
            session->sendNote<CoreNote>(CoreNoteType::SteppingEnabled);
        }
    });
    //            observe(core_.save_detail_request, [w_session]() {
    //                if(SessionPtr session = w_session.lock()) {
    //                    session->sendNote<CoreNote>(CoreNoteType::SaveDetailRequest);
    //                }
    //            });
    //            observe(core_.load_detail_request, [w_session]() {
    //                if(SessionPtr session = w_session.lock()) {
    //                    session->sendNote<CoreNote>(CoreNoteType::LoadDetailRequest);
    //                }
    //            });
    observe(core_.notification, [w_session](const Notification& n) {
        if (SessionPtr session = w_session.lock()) {
            session->sendNote<CoreNote>(CoreNoteType::Notification, n);
        }
    });

    // graphs and nodes
    GraphServerPtr graph_server = std::make_shared<GraphServer>(session);
    graph_servers_[session.get()] = graph_server;
    graph_server->startObservingGraph(core_.getRoot());

    session->start();

    std::unique_lock<std::recursive_mutex> lock(session_mutex_);
    sessions_.push_back(session);
}

void TcpServer::stopSession(Session* session)
{
    std::unique_lock<std::recursive_mutex> lock(session_mutex_);
    auto pos = std::find(sessions_.begin(), sessions_.end(), session->shared_from_this());
    if (pos != sessions_.end()) {
        sessions_.erase(pos);

        graph_servers_.erase(session);
    }
}

void TcpServer::handlePacket(const SessionPtr& session, const StreamableConstPtr& packet)
{
    if (CommandConstPtr cmd = std::dynamic_pointer_cast<Command const>(packet)) {
        if (!cmd) {
            StreamableConstPtr response = std::make_shared<Feedback>("unknown command received: ");
            session->write(response);
        } else {
            core_.getCommandDispatcher()->execute(cmd->cloneAs<Command>());
        }

    } else if (RequestConstPtr request = std::dynamic_pointer_cast<Request const>(packet)) {
        StreamableConstPtr response;
        try {
            response = request->execute(session, core_);

        } catch (const std::exception& e) {
            response = std::make_shared<Feedback>(std::string("Request has thrown an exception: ") + e.what(), request->getRequestID());

        } catch (...) {
            response = std::make_shared<Feedback>(std::string("Request has failed with unkown cause."), request->getRequestID());
        }

        if (!response) {
            response = std::make_shared<Feedback>(std::string("Request failed to produce a response"), request->getRequestID());
        }

        session->write(response);

    } else {
        session->write("packet with unknown type received");
    }
}

namespace tcpimpl
{
bool g_killed = false;
void sighandler(const boost::system::error_code& error, int signal_number)
{
    if (signal_number == SIGINT) {
        g_killed = true;
    }
}

}  // namespace tcpimpl

void TcpServer::spin()
{
    do_accept();

    boost::asio::signal_set sig_handler(io_service_, SIGINT);
    sig_handler.async_wait(tcpimpl::sighandler);

    while (running_) {
        try {
            io_service_.run_one();
        } catch (const std::exception& e) {
            std::cerr << "the tcp server has thrown an exception: " << e.what() << std::endl;
        }

        if (tcpimpl::g_killed) {
            std::cerr << "the tcp server has been interrupted" << std::endl;
            return;
        }
    }
}

void TcpServer::start()
{
    running_ = true;

    if (spin_thread_) {
        worker_thread_ = std::thread([this]() { spin(); });
    } else {
        spin();
    }
}

void TcpServer::stop()
{
    if (running_) {
        running_ = false;

        {
            std::unique_lock<std::recursive_mutex> lock(session_mutex_);

            for (const SessionPtr& session : sessions_) {
                if (session) {
                    session->stop();
                }
            }
            sessions_.clear();
        }

        io_service_.stop();

        if (std::this_thread::get_id() != worker_thread_.get_id()) {
            if (spin_thread_ && worker_thread_.joinable()) {
                try {
                    worker_thread_.join();
                } catch (const std::runtime_error& e) {
                    std::cerr << "failed to join server thread: " << e.what() << std::endl;
                }
            }
        }
    }
}

bool TcpServer::isRunning() const
{
    return running_;
}
