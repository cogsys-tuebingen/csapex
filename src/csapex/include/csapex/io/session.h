#ifndef SESSION_H
#define SESSION_H

/// PROJECT
#include <csapex/core/core_fwd.h>
#include <csapex/serialization/serialization_buffer.h>
#include <csapex/model/observer.h>

/// SYSTEM
#include <boost/asio.hpp>

namespace csapex
{

class Session : public Observer, public std::enable_shared_from_this<Session>
{
public:
  Session(boost::asio::ip::tcp::socket socket, CsApexCorePtr core);
  ~Session();

  void start();
  void stop();

public:
  slim_signal::Signal<void()> started;
  slim_signal::Signal<void()> stopped;

private:
  void read_async();

  void write_packet(SerializationBuffer &buffer);
  void write_synch(const std::string &message);
  void write_asynch(const std::string &message);


  boost::asio::ip::tcp::socket socket_;
  CsApexCorePtr core_;

  enum { max_length = 1024 };
  SerializationBuffer data_;

  bool live_;
};

}

#endif // SESSION_H
