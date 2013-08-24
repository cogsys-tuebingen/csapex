#ifndef CMP_STATE_PUBLISHER_HPP
#define CMP_STATE_PUBLISHER_HPP
#include <boost/any.hpp>

class CMPStatePublisher {
public:
    typedef boost::shared_ptr<CMPStatePublisher> Ptr;

    CMPStatePublisher()
    {
    }

    template<typename T>
    void publish(const T &data)
    {
        boost::any to_puslish = data;
        publish_impl(to_puslish);
    }

protected:
    virtual void publish_impl(const boost::any &to_publish)
    {
    }
};

#endif // CMP_STATE_PUBLISHER_HPP
