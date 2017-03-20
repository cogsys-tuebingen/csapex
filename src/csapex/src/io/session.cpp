/// HEADER
#include <csapex/io/session.h>

/// PROJECT
#include <csapex/io/io_fwd.h>
#include <csapex/core/csapex_core.h>
#include <csapex/command/command.h>
#include <csapex/io/request.h>
#include <csapex/io/response.h>
#include <csapex/io/feedback.h>
#include <csapex/serialization/serialization_buffer.h>
#include <csapex/serialization/packet_serializer.h>

/// SYSTEM
#include <csapex/utility/error_handling.h>
#include <iostream>

using namespace csapex;
using boost::asio::ip::tcp;

Session::Session(tcp::socket socket)
    : socket_(std::move(socket)),
      next_request_id_(1),
      live_(false)
{
}

Session::~Session()
{
    if(live_) {
        socket_.close();
    }
}

void Session::start()
{
    live_ = true;
    started();
    read_async();
}

void Session::stop()
{
    live_ = false;
    stopped();

    std::unique_lock<std::recursive_mutex> lock(open_requests_mutex_);
    for(auto pair : open_requests_) {
        std::promise<ResponseConstPtr>* open_requests = pair.second;
        open_requests->set_value(nullptr);
    }
}


ResponseConstPtr Session::sendRequest(RequestConstPtr request)
{
    request->overwriteRequestID(next_request_id_++);

    write(request);

    std::promise<ResponseConstPtr> promise;

    {
        std::unique_lock<std::recursive_mutex> lock(open_requests_mutex_);
        open_requests_[request->getRequestID()] = &promise;
    }

    std::future<ResponseConstPtr> future = promise.get_future();

    future.wait();

    if(ResponseConstPtr response = future.get()) {
        return response;
    }

    return nullptr;
}

void Session::write(const SerializableConstPtr &packet)
{
    SerializationBuffer buffer = PacketSerializer::serializePacket(packet);
    write_packet(buffer);
}

void Session::write(const std::string &message)
{
    write(std::make_shared<Feedback>(message));
}

void Session::read_async()
{
    if(!live_) {
        return;
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
                uint8_t message_length(message_data->at(0));

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
                                }
                            }

                        } else if(ResponseConstPtr response = std::dynamic_pointer_cast<Response const>(serial)) {
                            std::cerr << "got response #" << response->getRequestID() << std::endl;

                            std::unique_lock<std::recursive_mutex> lock(open_requests_mutex_);
                            auto it = open_requests_.find(response->getRequestID());
                            if(it != open_requests_.end()) {
                                std::promise<ResponseConstPtr>* promise = it->second;
                                promise->set_value(response);
                                open_requests_.erase(it);
                            }
                        } else {
                            packet_received(serial);
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

        std::cerr << "sending:\n" << buffer.toString() << std::endl;

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
