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
#include <csapex/io/protcol/parameter_changed.h>
#include <csapex/io/session.h>

using boost::asio::ip::tcp;
using namespace csapex;


bool g_running;

void siginthandler(int param)
{
    g_running = false;
}


class DemoClient
{
public:
    DemoClient(int argc, char* argv[])
        : s(io_service), resolver(io_service)
    {
        signal(SIGINT, siginthandler);

        std::thread spinner;

        try
        {
            boost::asio::connect(s, resolver.resolve({argv[1], argv[2]}));

            session = std::make_shared<Session>(std::move(s));

            //        readPacket(s);
            session->packet_received.connect([this](SerializableConstPtr packet){
                handlePacket(packet);
            });

            session->start();

            g_running = true;
            spinner = std::thread([&](){
                while(g_running) {
                    io_service.run();
                }
            });

            session->stopped.connect([&](){
                std::cerr << "The server disconnected." << std::endl;
                g_running = false;
                io_service.stop();
            });

            settings_= std::make_shared<SettingsRemote>(session);
            try {
                param::ParameterPtr p = settings_->get("access-test");
                std::cerr << p->as<std::string>() << std::endl;
                apex_assert_equal_hard(std::string("access granted."), p->as<std::string>());

                p->parameter_changed.connect([](param::Parameter*p) {
                    std::cerr << "test observer has changed to " << p->as<std::string>() << std::endl;
                });


                param::ParameterPtr test_observer = settings_->get("test-observer");
                std::cerr << test_observer->as<std::string>() << std::endl;
                apex_assert_equal_hard(std::string("initialized"), test_observer->as<std::string>());

                test_observer->set<std::string>("change request");
                apex_assert_equal_hard(std::string("change request"), test_observer->as<std::string>());



            } catch(const std::out_of_range& e) {
                std::cerr << "no access to server settings" << std::endl;
                std::exit(1);
            } catch(const std::exception& e) {
                std::cerr << "an unknown error occurred: " << e.what()  << std::endl;
                std::exit(2);
            }

            while(g_running) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));

                param::ParameterPtr test_observer = settings_->get("test-observer");
                std::cerr << test_observer->as<std::string>() << std::endl;
                apex_assert_equal_hard(std::string("change has been processed"), test_observer->as<std::string>());
            }

            csapex::command::Quit::Ptr quit = std::make_shared<csapex::command::Quit>();
            session->write(quit);
        }
        catch (const csapex::HardAssertionFailure& e)
        {
            std::cerr << "Assertion failed: " << e.what() << "\n";
            std::exit(1);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Exception: " << e.what() << "\n";
        }

        spinner.join();
    }


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

                    } else if(auto parameter_change = std::dynamic_pointer_cast<ParameterChanged const>(broadcast)) {
                        AUUID uuid = parameter_change->getUUID();
                        std::cerr << "parameter " << uuid << " changed" << std::endl;
                        if(uuid.global()) {
                            param::ParameterPtr p = settings_->get(uuid.globalName());
                            if(p->set_unsafe(parameter_change->getValue())) {
                                p->triggerChange();
                                std::cerr << "setting " << uuid.globalName() << " changed to " << p->toString() << std::endl;
                            }
                        } else {
                            //TODO: implement for general parameter
                        }
                    }
                }
            }
        }
    }


private:
    boost::asio::io_service io_service;

    tcp::socket s;
    tcp::resolver resolver;

    SessionPtr session;

    std::shared_ptr<Settings> settings_;
};


int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        std::cerr << "Usage: demo_client <host> <port>\n";
        return 1;
    }

    DemoClient client(argc, argv);

    return 0;
}
