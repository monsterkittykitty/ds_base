#include <ros/ros.h>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>

class DsConnection
{
public:
  DsConnection();
  ~DsConnection();

  // Factory method
  static DsConnection* create(int type);

  virtual void send(boost::shared_ptr<std::string>,
		    const boost::system::error_code&,
		    std::size_t) = 0;
  
  virtual void receive(boost::function<void()> callback) = 0;
};
