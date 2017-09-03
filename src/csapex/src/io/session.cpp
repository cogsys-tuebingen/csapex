/// HEADER
#include <csapex/io/session.h>

/// PROJECT
#include <csapex/io/io_fwd.h>
#include <csapex/core/csapex_core.h>
#include <csapex/io/request.h>
#include <csapex/io/response.h>
#include <csapex/io/feedback.h>
#include <csapex/serialization/serialization_buffer.h>
#include <csapex/serialization/packet_serializer.h>
#include <csapex/io/broadcast_message.h>
#include <csapex/io/raw_message.h>

/// SYSTEM
#include <csapex/utility/error_handling.h>
#include <iostream>

using namespace csapex;
using boost::asio::ip::tcp;

Session::Session(tcp::socket socket)
    : socket_(std::move(socket)),
      next_request_id_(1),
      running_(false),
      live_(false)
{
}

Session::~Session()
{
    {
        std::unique_lock<std::recursive_mutex> running_lock(running_mutex_);
        if(running_) {
            stop();
        }

        apex_assert_hard(!packet_handler_thread_.joinable());
    }

    socket_.close();
}

void Session::start()
{
    {
        std::unique_lock<std::recursive_mutex> running_lock(running_mutex_);
        apex_assert_hard(!running_);
        running_ = true;
    }
    started();

    packet_handler_thread_ = std::thread([this](){
        live_ = true;

        while(running_) {
            std::unique_lock<std::recursive_mutex> packet_lock(packets_mutex_);
            while(running_ && packets_.empty()) {
                packets_available_.wait_for(packet_lock, std::chrono::milliseconds(100));
            }

            while(running_ && !packets_.empty()) {
                SerializableConstPtr packet = packets_.front();
                packets_.pop_front();
                packet_lock.unlock();

                try {

                    switch(packet->getPacketType()) {
                    case BroadcastMessage::PACKET_TYPE_ID:
                        if(BroadcastMessageConstPtr broadcast = std::dynamic_pointer_cast<BroadcastMessage const>(packet)) {
                            broadcast_received(broadcast);
                            break;
                        }
                    case RawMessage::PACKET_TYPE_ID:
                        if(RawMessageConstPtr raw_message = std::dynamic_pointer_cast<RawMessage const>(packet)) {
                            auto& signal = raw_packet_received(raw_message->getUUID());
                            signal(raw_message);
                        }
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
        live_ = false;
    });

    read_async();
}

void Session::stop()
{
    apex_assert_hard(packet_handler_thread_.get_id() != std::this_thread::get_id());

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

    stopped();

    if(live_) {
        packet_handler_thread_.join();
    }
    apex_assert_hard(!live_);
}


ResponseConstPtr Session::sendRequest(RequestConstPtr request)
{
    if(live_) {
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
            return response;
        }

    }
    return nullptr;
}

void Session::write(const SerializableConstPtr &packet)
{
    if(live_) {
        try {
            SerializationBuffer buffer = PacketSerializer::serializePacket(packet);
            write_packet(buffer);
        } catch(const std::exception& e) {
            std::cerr << "error writing packet: " << e.what() << std::endl;
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

    std::shared_ptr<SerializationBuffer> message_data = std::make_shared<SerializationBuffer>();
    boost::asio::async_read(socket_, boost::asio::buffer(&message_data->at(0), SerializationBuffer::HEADER_LENGTH),
                            [this, message_data](boost::system::error_code ec, std::size_t reply_length){
        if ((ec  == boost::asio::error::eof) || (ec == boost::asio::error::connection_reset)) {
            // disconnect
            stop();

        } else if(reply_length > 0) {
            // payload received
            if(reply_length == SerializationBuffer::HEADER_LENGTH) {
                message_data->seek(0);
                uint32_t message_length;
                *message_data >> message_length;

                if(message_length <= SerializationBuffer::HEADER_LENGTH) {
                    std::cerr << "got illegal message of length " << (int) message_length << std::endl;

                } else {
                    message_data->resize(message_length, ' ');
                    reply_length = boost::asio::read(socket_, boost::asio::buffer(&message_data->at(SerializationBuffer::HEADER_LENGTH), message_length - SerializationBuffer::HEADER_LENGTH));
                    apex_assert_equal_hard((int) reply_length, ((int) (message_length - SerializationBuffer::HEADER_LENGTH)));

                    SerializablePtr serial = PacketSerializer::deserializePacket(*message_data);

                    if(serial) {
                        if(FeedbackConstPtr feedback = std::dynamic_pointer_cast<Feedback const>(serial)) {
                            std::cerr << feedback->getMessage() << std::endl;
                            if(feedback->getRequestID() != 0) {
                                std::unique_lock<std::recursive_mutex> lock(open_requests_mutex_);
                                auto it = open_requests_.find(feedback->getRequestID());
                                if(it != open_requests_.end()) {
                                    std::promise<ResponseConstPtr>* promise = it->second;
                                    promise->set_value(nullptr);
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
                                promise->set_value(response);
                                open_requests_.erase(it);

                            } else {
                                std::cerr << "got response for unknown request " << (int) response->getRequestID() << std::endl;
                            }

                        } else {
                            std::unique_lock<std::recursive_mutex> packet_lock(packets_mutex_);
                            packets_.push_back(serial);
                            packets_available_.notify_all();
                        }
                    }
                }

            } else {
                std::cerr << "got illegal header of length " << (int) reply_length << std::endl;
            }
        }
        read_async();
    });
}

void Session::write_packet(SerializationBuffer &buffer)
{
    try {
        buffer.finalize();

        apex_assert_hard(socket_.is_open());

//                std::cerr << "sending:\n" << buffer.toString() << std::endl;

        boost::asio::async_write(socket_, boost::asio::buffer(buffer, buffer.size()),
                                 [](boost::system::error_code /*ec*/, std::size_t /*length*/){});

    } catch(const std::exception& e) {
        std::cerr << "the session has thrown an exception: " << e.what() << std::endl;
        stop();
    } catch(...) {
        std::cerr << "the session has crashed with an unknown cause." << std::endl;
        stop();
    }
}


slim_signal::Signal<void(RawMessageConstPtr)>& Session::raw_packet_received(const AUUID& uuid)
{
    auto& res = auuid_to_signal_[uuid];
    if(res == nullptr) {
        res.reset(new slim_signal::Signal<void(RawMessageConstPtr)>);
    }
    return *res;
}
