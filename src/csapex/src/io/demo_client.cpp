#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>

#include <csapex/utility/exceptions.h>
#include <csapex/command/quit.h>
#include <csapex/io/broadcast_message.h>
#include <csapex/io/protcol/notification_message.h>
#include <csapex/serialization/packet_serializer.h>
#include <csapex/core/settings/settings_remote.h>
#include <csapex/io/session.h>


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

        SessionPtr session = std::make_shared<Session>(std::move(s));

//        readPacket(s);
        session->packet_received.connect([](SerializableConstPtr packet){
            handlePacket(packet);
        });

        session->start();


//        SettingsRemote remote_settings(session);
//        Settings& settings = remote_settings;
//        try {
//            param::ParameterPtr p = settings.get("access-test");
//            std::cerr << p->as<std::string>() << std::endl;
//            std::exit(0);
//        } catch(const std::out_of_range& e) {
//            std::cerr << "no access to server settings" << std::endl;
//            std::exit(0);
//        }
//
//        io_service.run();

        csapex::command::Quit::Ptr quit = std::make_shared<csapex::command::Quit>();

        session->write(quit);

        io_service.run();
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
