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
#include <csapex/io/protcol/core_requests.h>

using boost::asio::ip::tcp;
using namespace csapex;

class DemoClient;

DemoClient* g_client = nullptr;

void siginthandler(int param);


class DemoClient
{
public:
    DemoClient(int argc, char* argv[])
        : socket(io_service), resolver(io_service), running(false)
    {
        g_client = this;

        signal(SIGINT, siginthandler);

        std::thread spinner;

        try
        {
            boost::asio::connect(socket, resolver.resolve({argv[1], argv[2]}));

            session = std::make_shared<Session>(std::move(socket));

            //        readPacket(s);
            session->packet_received.connect([this](StreamableConstPtr packet){
                handlePacket(packet);
            });

            session->start();

            running = true;
            spinner = std::thread([&](){
                while(running) {
                    io_service.run();
                }
            });

            session->stopped.connect([&](){
                std::cerr << "The server disconnected." << std::endl;
                stop();
            });

            settings_= std::make_shared<SettingsRemote>(session);

//            try {
//                param::ParameterPtr p = settings_->get("access-test");
//                std::cerr << p->as<std::string>() << std::endl;
//                apex_assert_equal_hard(std::string("access granted."), p->as<std::string>());

//                p->parameter_changed.connect([](param::Parameter*p) {
//                    std::cerr << "test observer has changed to " << p->as<std::string>() << std::endl;
//                });


//                param::ParameterPtr test_observer = settings_->get("test-observer");
//                std::cerr << test_observer->as<std::string>() << std::endl;
//                apex_assert_equal_hard(std::string("initialized"), test_observer->as<std::string>());

//                test_observer->set<std::string>("change request");
//                apex_assert_equal_hard(std::string("change request"), test_observer->as<std::string>());


//                apex_assert_hard(!settings_->knows("foo-bar_baz"));
//                settings_->set<std::string>("foo-bar_baz", "lorem-ipsum");
//                apex_assert_hard(settings_->knows("foo-bar_baz"));
//                apex_assert_equal_hard("lorem-ipsum", settings_->get<std::string>("foo-bar_baz"));


//            } catch(const std::out_of_range& e) {
//                std::cerr << "no access to server settings" << std::endl;
//                std::exit(1);
//            } catch(const std::exception& e) {
//                std::cerr << "an unknown error occurred: " << e.what()  << std::endl;
//                std::exit(2);
//            }

//            int test_iterations = 3;
//            while(running) {
//                std::this_thread::sleep_for(std::chrono::milliseconds(500));

//                param::ParameterPtr test_observer = settings_->get("test-observer");
//                std::cerr << test_observer->as<std::string>() << std::endl;
//                apex_assert_equal_hard(std::string("change has been processed"), test_observer->as<std::string>());

//                apex_assert_equal_hard(std::string("change has been processed"), settings_->get<std::string>("test-observer"));

//                if(--test_iterations <= 0) {
//                    stop();
//                }
//            }

            while(running) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }


//            session->sendRequest<CoreRequests>(CoreRequests::CoreRequestType::CoreSave);

//            csapex::command::Quit::Ptr quit = std::make_shared<csapex::command::Quit>();
//            session->write(quit);
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

        io_service.stop();

        spinner.join();
    }

    void stop()
    {
        running = false;
    }

    void handlePacket(StreamableConstPtr packet)
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


private:
    boost::asio::io_service io_service;

    tcp::socket socket;
    tcp::resolver resolver;

    SessionPtr session;

    std::shared_ptr<Settings> settings_;

    bool running;
};

void siginthandler(int param)
{
    if(g_client) {
        g_client->stop();
    }
}

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
