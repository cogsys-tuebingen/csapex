/// HEADER
#include <csapex/io/session.h>

/// PROJECT
#include <csapex/io/io_fwd.h>
#include <csapex/core/csapex_core.h>
#include <csapex/command/command.h>
#include <csapex/io/request.h>
#include <csapex/io/response.h>
#include <csapex/serialization/packet_serializer.h>

/// SYSTEM
#include <csapex/utility/error_handling.h>

using namespace csapex;
using boost::asio::ip::tcp;

Session::Session(tcp::socket socket)
    : socket_(std::move(socket)),
      live_(false)
{
    data_.resize(max_length);
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
}


ResponseConstPtr Session::sendRequest(RequestConstPtr request)
{
    std::cerr << "sending request" << std::endl;;
    write(request);
    std::cerr << "waiting for answer" << std::endl;
    SerializableConstPtr result = read();

    std::cerr << "got resonse" << std::endl;;
    if(ResponseConstPtr response = std::dynamic_pointer_cast<Response const>(result)) {
        return response;
    }

//    TODO: don't  block in write and read, rather use ids for messages
//            -> currently notifications can be received in between request and response!
    throw std::logic_error("request didn't return a response.");
}

void Session::write(const SerializableConstPtr &packet)
{
    SerializationBuffer buffer = PacketSerializer::serializePacket(packet);
    write_packet(buffer);
}

void Session::write(const std::string &message)
{
    write_synch(message);
}

void Session::read_async()
{
    if(!live_) {
        return;
    }


    std::shared_ptr<SerializationBuffer> message_data = std::make_shared<SerializationBuffer>();
//    std::cerr << "async read" << std::endl;
//    size_t reply_length = boost::asio::read(s, boost::asio::buffer(&message_data->at(0), SerializationBuffer::HEADER_LENGTH));
//    std::cerr << reply_length << std::endl;

    //s.async_read_some(boost::asio::buffer(&message_data->at(0), SerializationBuffer::HEADER_LENGTH),
      //                      [&s, message_data](boost::system::error_code ec, std::size_t reply_length){
    boost::asio::async_read(socket_, boost::asio::buffer(&message_data->at(0), SerializationBuffer::HEADER_LENGTH),
                            [this, message_data](boost::system::error_code ec, std::size_t reply_length){
        if ((ec  == boost::asio::error::eof) || (ec == boost::asio::error::connection_reset)) {
            // disconnect
            stop();

        } else if(reply_length > 0) {
            // payload received
            if(reply_length == SerializationBuffer::HEADER_LENGTH) {
                uint8_t message_length(message_data->at(0));

                std::cerr << "next message: " << (int) message_length << std::endl;

                message_data->resize(message_length, ' ');
                reply_length = boost::asio::read(socket_, boost::asio::buffer(&message_data->at(SerializationBuffer::HEADER_LENGTH), message_length - SerializationBuffer::HEADER_LENGTH));
                apex_assert_equal_hard((int) reply_length, ((int) (message_length - SerializationBuffer::HEADER_LENGTH)));

                SerializablePtr serial = PacketSerializer::deserializePacket(*message_data);

                if(serial) {
                    packet_received(serial);
                }
            }
        }
        read_async();
    });
}


SerializableConstPtr Session::read()
{
    apex_assert_hard(live_);

    std::shared_ptr<SerializationBuffer> message_data = std::make_shared<SerializationBuffer>();

    // TODO: implement timeout
    boost::system::error_code ec;
    while (!ec) {
        size_t reply_length = boost::asio::read(socket_, boost::asio::buffer(&message_data->at(0), SerializationBuffer::HEADER_LENGTH), ec);

        if(reply_length > 0) {
            uint8_t message_length(message_data->at(0));

            message_data->resize(message_length, ' ');
            reply_length = boost::asio::read(socket_, boost::asio::buffer(&message_data->at(SerializationBuffer::HEADER_LENGTH), message_length - SerializationBuffer::HEADER_LENGTH));
            apex_assert_equal_hard((int) reply_length, ((int) (message_length - SerializationBuffer::HEADER_LENGTH)));

            return PacketSerializer::deserializePacket(*message_data);
        }
    }

    return nullptr;
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


void Session::write_synch(const std::string& message)
{
    std::cerr << "writing " << message << std::endl;
    boost::system::error_code ignored_error;

    char size_msg[] = { (char) message.size() };
    boost::asio::write(socket_, boost::asio::buffer(size_msg),
                       boost::asio::transfer_all(), ignored_error);

    boost::asio::write(socket_, boost::asio::buffer(message),
                       boost::asio::transfer_all(), ignored_error);
}

void Session::write_asynch(const std::string& message)
{
    if(!live_) {
        return;
    }

    auto self(shared_from_this());
    char size_msg[] = { (char) message.size() };
    boost::asio::async_write(socket_, boost::asio::buffer(size_msg, 1),
                             [this, message, self](boost::system::error_code ec, std::size_t /*length*/)
    {
        if (!ec)
        {
            boost::asio::async_write(socket_, boost::asio::buffer(message.data(), message.size()),
                                     [this, self](boost::system::error_code ec, std::size_t /*length*/)
            {
                if (!ec)
                {
                    //            do_read();
                }
            });
        }
    });
}
