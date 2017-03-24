#ifndef SESSION_H
#define SESSION_H

/// PROJECT
#include <csapex/core/core_fwd.h>
#include <csapex/serialization/serialization_fwd.h>
#include <csapex/model/observer.h>
#include <csapex/io/io_fwd.h>

/// SYSTEM
#include <boost/asio.hpp>
#include <deque>
#include <future>

namespace csapex
{

class Session : public Observer, public std::enable_shared_from_this<Session>
{
public:
  Session(boost::asio::ip::tcp::socket socket);
  ~Session();

  void start();
  void stop();

  void write(const SerializableConstPtr &packet);
  void write(const std::string &message);

  ResponseConstPtr sendRequest(RequestConstPtr request);

  template <typename RequestWrapper>
  std::shared_ptr<typename RequestWrapper::ResponseT const>
  sendRequest(std::shared_ptr<typename RequestWrapper::RequestT const> request)
  {
      return std::dynamic_pointer_cast<typename RequestWrapper::RequestT const>(sendRequest(request));
  }

  template <typename RequestWrapper, typename... Args>
  std::shared_ptr<typename RequestWrapper::ResponseT const>
  sendRequest(Args&&... args)
  {
      return std::dynamic_pointer_cast<typename RequestWrapper::ResponseT const>(sendRequest(std::make_shared<typename RequestWrapper::RequestT>(std::forward<Args>(args)...)));
  }

public:
  slim_signal::Signal<void()> started;
  slim_signal::Signal<void()> stopped;

  slim_signal::Signal<void(SerializableConstPtr)> packet_received;

private:
  void read_async();

  void write_packet(SerializationBuffer &buffer);

  std::thread packet_handler_thread_;
  boost::asio::ip::tcp::socket socket_;

  uint8_t next_request_id_;


  std::recursive_mutex packets_mutex_;
  std::condition_variable_any packets_available_;
  std::deque<SerializableConstPtr> packets_;

  std::recursive_mutex open_requests_mutex_;
  std::map<uint8_t, std::promise<ResponseConstPtr>*> open_requests_;

  std::recursive_mutex running_mutex_;
  std::atomic<bool> running_;
  std::atomic<bool> live_;
};

}

#endif // SESSION_H
