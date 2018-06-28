/// HEADER
#include <csapex/io/session.h>

/// PROJECT
#include <csapex/io/remote_io_fwd.h>
#include <csapex/core/csapex_core.h>
#include <csapex/io/request.h>
#include <csapex/io/response.h>
#include <csapex/io/feedback.h>
#include <csapex/io/note.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/serialization/packet_serializer.h>
#include <csapex/io/broadcast_message.h>
#include <csapex/io/raw_message.h>
#include <csapex/io/protcol/core_notes.h>
#include <csapex/io/channel.h>
#include <csapex/utility/thread.h>
#include <csapex/utility/exceptions.h>

/// SYSTEM
#include <csapex/utility/error_handling.h>
#include <iostream>

using namespace csapex;
using boost::asio::ip::tcp;

Session::Session(Socket socket, const std::string &name)
    : socket_(new Socket(std::move(socket))),
      next_request_id_(1),
      running_(false),
      is_live_(false),
      was_live_(false),
      name_(name),
      is_valid_(true)
{
}

Session::Session(const std::string& name)
    : next_request_id_(1),
      running_(false),
      is_live_(false),
      was_live_(false),
      name_(name),
      is_valid_(true)
{
}

Session::~Session()
{
    is_valid_ = false;
    {
        std::unique_lock<std::recursive_mutex> running_lock(running_mutex_);
        if(running_) {
            running_ = false;
            packet_handler_thread_.join();
        }
    }

    if(socket_) {
        boost::system::error_code ec;
        socket_->shutdown(boost::asio::ip::tcp::socket::shutdown_send, ec);
        if (ec && ec != boost::asio::error::not_connected) {
            std::cerr << "ERROR: cannot shutdown server connection: " << ec.message() << std::endl;
        } else if(socket_->is_open()) {
            socket_->close();
        }
    }
}

void Session::run()
{

}

bool Session::isRunning() const
{
    return running_;
}

void Session::shutdown()
{

}

void Session::start()
{
    {
        std::unique_lock<std::recursive_mutex> running_lock(running_mutex_);
        apex_assert_hard(!running_);
        running_ = true;
    }
    started(this);

    packet_handler_thread_ = std::thread([this](){
        csapex::thread::set_name(name_.c_str());
        is_live_ = true;
        was_live_ = true;

        try {
            mainLoop();
        }
        catch(const std::exception& e) {
            std::cerr << "there was an error in the session: " << e.what() << std::endl;
        }
        catch(const csapex::Failure& e) {
            std::cerr << "there was a failure in the session: " << e.what() << std::endl;
        }
        catch(...) {
            std::cerr << "there was an unknown error in the session" << std::endl;
        }

        is_live_ = false;
    });

    read_async();
}

void Session::mainLoop()
{
    while(running_) {
        std::unique_lock<std::recursive_mutex> packet_lock(packets_mutex_);
        while(running_ && (packets_received_.empty() && packets_to_send_.empty())) {
            packets_available_.wait_for(packet_lock, std::chrono::milliseconds(100));
        }

        // so as not to starve the clients, we limit the max amount of packets to send
        // before receiving other packets
        const int max_operations_per_iteration = 32;
        for(int i = 0;
            i < max_operations_per_iteration && running_ && !packets_to_send_.empty();
            ++i)
        {
            StreamableConstPtr packet = packets_to_send_.front();
            packets_to_send_.pop_front();
            packet_lock.unlock();

            SerializationBuffer buffer = PacketSerializer::serializePacket(packet);
            write_packet(buffer);

            packet_lock.lock();
        }

        for(int i = 0;
            i < max_operations_per_iteration && running_ && !packets_received_.empty();
            ++i)
        {
            StreamableConstPtr packet = packets_received_.front();
            packets_received_.pop_front();
            packet_lock.unlock();

            try {

                switch(packet->getPacketType()) {
                case BroadcastMessage::PACKET_TYPE_ID:
                    if(BroadcastMessageConstPtr broadcast = std::dynamic_pointer_cast<BroadcastMessage const>(packet)) {
                        broadcast_received(broadcast);
                    }
                    break;
                case RawMessage::PACKET_TYPE_ID:
                    if(RawMessageConstPtr raw_message = std::dynamic_pointer_cast<RawMessage const>(packet)) {
                        auto& signal = raw_packet_received(raw_message->getUUID()); // move this to node server
                        signal(raw_message);
                    }
                    break;
                case io::Note::PACKET_TYPE_ID:
                    if(io::NoteConstPtr note = std::dynamic_pointer_cast<io::Note const>(packet)) {
                        auto pos = channels_.find(note->getAUUID());
                        if(pos != channels_.end()) {
                            io::ChannelPtr channel = pos->second;
                            channel->handleNote(note);
                        }
                    }
                    break;
                default:
                    packet_received(packet);
                    break;
                }

            } catch(...) {
                // silent death
            }

            packet_lock.lock();
        }
    }
}

void Session::stop()
{
    apex_assert_hard(packet_handler_thread_.get_id() != std::this_thread::get_id());

    if(is_valid_) {
        stopped(this);
    }

    std::unique_lock<std::recursive_mutex> running_lock(running_mutex_);

//    if(!live_) {
//        return;
//    }
    running_ = false;

    std::unique_lock<std::recursive_mutex> lock(open_requests_mutex_);
    for(auto pair : open_requests_) {
        std::promise<ResponseConstPtr>* open_requests = pair.second;
        open_requests->set_value(nullptr);
    }

    running_lock.unlock();

    if(is_live_) {
        packet_handler_thread_.join();
    }
    apex_assert_hard(!is_live_);
}

void Session::stopForced()
{
    if(packet_handler_thread_.get_id() != std::this_thread::get_id()) {
        stop();
    } else {
        throw std::runtime_error("stopping in an active session");
    }
}


ResponseConstPtr Session::sendRequest(RequestConstPtr request)
{
    if(is_live_) {
        request->overwriteRequestID(next_request_id_++);

        std::promise<ResponseConstPtr> promise;

        {
            std::unique_lock<std::recursive_mutex> lock(open_requests_mutex_);
            open_requests_[request->getRequestID()] = &promise;
        }

        write(request);

        std::future<ResponseConstPtr> future = promise.get_future();

        future.wait();

        if(ResponseConstPtr response = future.get()) {
            apex_assert_hard(response);
            return response;
        }
        apex_fail(std::string("The request ") + std::to_string(request->getRequestID()) + " failed to produce a response");


    } else {
        if(was_live_) {
            throw NoConnectionException();
        }
    }

    return nullptr;
}

void Session::sendNote(io::NoteConstPtr note)
{
    try {
        write(note);
    } catch(const NoConnectionException& nce) {
        // ignore notes sent
        std::cerr << "cannot send note " <<
                     note->getType() <<
                     " via a closed session" << std::endl;
    }
}

void Session::write(const StreamableConstPtr &packet)
{
    if(is_live_) {
        {
            if(packet_handler_thread_.get_id() != std::this_thread::get_id()) {
                std::unique_lock<std::recursive_mutex> packet_lock(packets_mutex_);
                packets_to_send_.push_back(packet);
            } else {
                SerializationBuffer buffer = PacketSerializer::serializePacket(packet);
                write_packet(buffer);
            }
        }

        packets_available_.notify_all();

    } else {
        if(was_live_) {
            throw NoConnectionException();
        }
    }
}

void Session::write(const std::string &message)
{
    write(std::make_shared<Feedback>(message));
}

void Session::read_async()
{
    {
        if(!running_) {
            return;
        }
    }

    SessionWeakPtr self = shared_from_this();

    std::shared_ptr<SerializationBuffer> message_data = std::make_shared<SerializationBuffer>();
    boost::asio::async_read(*socket_, boost::asio::buffer(&message_data->at(0), SerializationBuffer::HEADER_LENGTH),
                            [this, message_data, self](boost::system::error_code ec, std::size_t reply_length){
        if (ec  == boost::asio::error::eof) {
            // do nothing
            return;

        } else if (ec == boost::asio::error::connection_reset) {
            // disconnect
            if(SessionPtr session = self.lock()) {
                stop();
            }
            return;

        } else if(reply_length > 0) {
            // payload received
            if(reply_length == SerializationBuffer::HEADER_LENGTH) {
                message_data->seek(0);
                uint32_t message_length;
                *message_data >> message_length;

                if(message_length > SerializationBuffer::HEADER_LENGTH) {
                    message_data->resize(message_length, ' ');
                    reply_length = boost::asio::read(*socket_, boost::asio::buffer(&message_data->at(SerializationBuffer::HEADER_LENGTH), message_length - SerializationBuffer::HEADER_LENGTH));
                    apex_assert_equal_hard((int) reply_length, ((int) (message_length - SerializationBuffer::HEADER_LENGTH)));


                    StreamablePtr serial = PacketSerializer::deserializePacket(*message_data);

                    if(serial) {
                        if(FeedbackConstPtr feedback = std::dynamic_pointer_cast<Feedback const>(serial)) {
                            std::cerr << feedback->getMessage() << std::endl;
                            if(feedback->getRequestID() != 0) {
                                std::unique_lock<std::recursive_mutex> lock(open_requests_mutex_);
                                auto it = open_requests_.find(feedback->getRequestID());
                                if(it != open_requests_.end()) {
                                    std::promise<ResponseConstPtr>* promise = it->second;
                                    promise->set_value(feedback);
                                    open_requests_.erase(it);

                                } else {
                                    std::cerr << "got feedback for unknown request " << (int) feedback->getRequestID() << std::endl;
                                }
                            }

                        } else if(ResponseConstPtr response = std::dynamic_pointer_cast<Response const>(serial)) {
                            //std::cerr << "got response #" << (int) response->getRequestID() << std::endl;

                            std::unique_lock<std::recursive_mutex> lock(open_requests_mutex_);
                            auto it = open_requests_.find(response->getRequestID());
                            if(it != open_requests_.end()) {
                                std::promise<ResponseConstPtr>* promise = it->second;
                                apex_assert_hard(response);
                                promise->set_value(response);
                                open_requests_.erase(it);

                            } else {
                                std::cerr << "got response for unknown request " << (int) response->getRequestID() << std::endl;
                            }

                        } else {
                            std::unique_lock<std::recursive_mutex> packet_lock(packets_mutex_);
                            packets_received_.push_back(serial);
                            packets_available_.notify_all();
                        }
                    } else {
                        std::cerr << "could not deserialize message of length " << (int) message_length << std::endl;
                    }
                } else {
                    std::cerr << "got illegal message of length " << (int) message_length << std::endl;
                }
            } else {
                std::cerr << "got illegal header of length " << (int) reply_length << std::endl;
            }
        } else {
            std::cerr << "got an illegal reply of length " << (int) reply_length << std::endl;
        }

        read_async();
    });
}

void Session::write_packet(SerializationBuffer &buffer)
{
    try {
        buffer.finalize();

        apex_assert_hard(socket_->is_open());
        //std::cerr << (long) this << " is sending:\n" << buffer.toString() << std::endl;

        std::size_t written_bytes = boost::asio::write(*socket_, boost::asio::buffer(buffer, buffer.size()));

        apex_assert_eq_hard(buffer.size(), written_bytes);

        //std::cerr << (long) this << " has sent " << written_bytes << " bytes" << std::endl;

    } catch(const std::exception& e) {
        std::cerr << "the session has thrown an exception: " << e.what() << std::endl;
        stopForced();
    } catch(...) {
        std::cerr << "the session has crashed with an unknown cause." << std::endl;
        stopForced();
    }
}

void Session::handleFeedback(const ResponseConstPtr &res)
{
    if(auto feedback = std::dynamic_pointer_cast<Feedback const>(res)) {
        throw Failure(feedback->getMessage());
    }
}

slim_signal::Signal<void (const StreamableConstPtr &)> &Session::raw_packet_received(const AUUID& uuid)
{
    auto& res = auuid_to_signal_[uuid];
    if(res == nullptr) {
        res.reset(new slim_signal::Signal<void(const StreamableConstPtr&)>);
    }
    return *res;
}

io::ChannelPtr Session::openChannel(const AUUID &name)
{
    auto pos = channels_.find(name);
    if(pos != channels_.end()) {
        return pos->second;
    }

    io::ChannelPtr channel = std::make_shared<io::Channel>(*this, name);
    channels_[name] = channel;
    return channel;
}
