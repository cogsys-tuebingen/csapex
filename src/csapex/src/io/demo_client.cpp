#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>

#include <csapex/utility/exceptions.h>
#include <csapex/command/quit.h>
#include <csapex/io/broadcast_message.h>
#include <csapex/io/protcol/notification_message.h>
#include <csapex/io/packet_serializer.h>
#include <csapex/core/settings.h>


using boost::asio::ip::tcp;
using namespace csapex;


boost::asio::io_service* g_io_service;

void handlePacket(SerializableConstPtr packet)
{
    if(packet) {
//                std::cout << "type=" << (int) serial->getPacketType() << std::endl;

        switch(packet->getPacketType()) {
        case BroadcastMessage::PACKET_TYPE_ID:
            if(BroadcastMessageConstPtr broadcast = std::dynamic_pointer_cast<BroadcastMessage const>(packet)) {
                if(auto notification_msg = std::dynamic_pointer_cast<NotificationMessage const>(broadcast)) {
                    Notification notification = notification_msg->getNotification();
                    std::cout << "$ " << notification.auuid << ": " << notification.message.str() << std::endl;
                }
            }
        }
    }
}


void readPacket(tcp::socket& s)
{
    std::shared_ptr<SerializationBuffer> message_data = std::make_shared<SerializationBuffer>();
    std::cerr << "async read" << std::endl;
//    size_t reply_length = boost::asio::read(s, boost::asio::buffer(&message_data->at(0), SerializationBuffer::HEADER_LENGTH));
//    std::cerr << reply_length << std::endl;

    //s.async_read_some(boost::asio::buffer(&message_data->at(0), SerializationBuffer::HEADER_LENGTH),
      //                      [&s, message_data](boost::system::error_code ec, std::size_t reply_length){
    boost::asio::async_read(s, boost::asio::buffer(&message_data->at(0), SerializationBuffer::HEADER_LENGTH),
                            [&s, message_data](boost::system::error_code ec, std::size_t reply_length){
        apex_assert_hard(reply_length == SerializationBuffer::HEADER_LENGTH);
        uint8_t message_length(message_data->at(0));

        std::cerr << "next message: " << (int) message_length << std::endl;

        message_data->resize(message_length, ' ');
        reply_length = boost::asio::read(s, boost::asio::buffer(&message_data->at(SerializationBuffer::HEADER_LENGTH), message_length - SerializationBuffer::HEADER_LENGTH));
        apex_assert_equal_hard((int) reply_length, ((int) (message_length - SerializationBuffer::HEADER_LENGTH)));

        SerializableConstPtr serial = PacketSerializer::deserializePacket(*message_data);

        //            std::cerr << "received\n" << message_data.toString() << std::endl;

        handlePacket(serial);

        readPacket(s);
    });
}
void siginthandler(int param)
{
    g_io_service->stop();
}

int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        std::cerr << "Usage: demo_client <host> <port>\n";
        return 1;
    }

    signal(SIGINT, siginthandler);

    try
    {
        boost::asio::io_service io_service;
        g_io_service = &io_service;

        tcp::socket s(io_service);
        tcp::resolver resolver(io_service);
        boost::asio::connect(s, resolver.resolve({argv[1], argv[2]}));

        Settings settings;
        std::cerr << settings.get("access-test", std::string("no access possible")) << std::endl;

        readPacket(s);

        io_service.run();

        csapex::command::Quit::Ptr quit = std::make_shared<csapex::command::Quit>();
        SerializationBuffer data = PacketSerializer::serializePacket(quit);
        boost::asio::write(s, boost::asio::buffer(data, data.size()));

    }
    catch (const csapex::HardAssertionFailure& e)
    {
        std::cerr << "Assertion failed: " << e.what() << "\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << "\n";
    }

    return 0;
}
