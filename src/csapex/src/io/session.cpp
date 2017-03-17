/// HEADER
#include <csapex/io/session.h>

/// PROJECT
#include <csapex/io/io_fwd.h>
#include <csapex/core/csapex_core.h>
#include <csapex/command/command.h>
#include <csapex/io/packet_serializer.h>
#include <csapex/io/protcol/notification_message.h>

/// SYSTEM
#include <csapex/utility/error_handling.h>

using namespace csapex;
using boost::asio::ip::tcp;

Session::Session(tcp::socket socket, CsApexCorePtr core)
    : socket_(std::move(socket)),
      core_(core),
      live_(false)
{
    data_.resize(max_length);

    observe(core_->notification, [this](const Notification& notification) {
        std::shared_ptr<NotificationMessage> msg = std::make_shared<NotificationMessage>(notification);
        SerializationBuffer buffer = PacketSerializer::serializePacket(msg);

        std::cerr << "notification!" << std::endl;
        write_packet(buffer);
    });
}

Session::~Session()
{

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

void Session::read_async()
{
    if(!live_) {
        return;
    }

    auto self(shared_from_this());
    socket_.async_read_some(boost::asio::buffer(data_, max_length),
                            [this, self](boost::system::error_code ec, std::size_t length)
    {
        if (!ec)
        {
            if(SerializablePtr packet = PacketSerializer::deserializePacket(data_)) {
                if(CommandPtr cmd = std::dynamic_pointer_cast<Command>(packet)) {
                    if(!cmd) {
                        write_synch("unknown command received");
                    } else {
                        write_synch(cmd->getDescription());
                        core_->getCommandDispatcher()->execute(cmd);
                    }
                } else {
                    write_synch("packet with unknown type received");
                }
            } else {
                write_synch("corrupt packet received");
            }
        }

        read_async();
    });
}

void Session::write_packet(SerializationBuffer &buffer)
{
    try {
        buffer.finalize();

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
