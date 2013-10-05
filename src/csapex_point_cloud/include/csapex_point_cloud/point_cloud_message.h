#ifndef POINT_CLOUD_MESSAGE_H
#define POINT_CLOUD_MESSAGE_H

/// PROJECT
#include <csapex/model/message.h>

/// SYSTEM
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/variant.hpp>
#include <boost/mpl/vector.hpp>

namespace csapex {
namespace connection_types {

/// ALL SUPPORTED POINT TYPES
typedef boost::mpl::vector<
pcl::PointXYZ,
pcl::PointXYZI,
pcl::PointXYZRGB
> PointCloudPointTypes;

template<class T>
struct add_point_cloud_ptr
{
    typedef typename pcl::PointCloud<T>::Ptr type;
};


struct PointCloudMessage : public Message
{
    template <typename T>
    struct Dispatch : public boost::static_visitor<void>
    {
        Dispatch(T* pc)
            : pc_(pc)
        {}

        template <typename PointCloudT>
        void operator () (PointCloudT cloud) const
        {
            typedef typename PointCloudT::element_type::PointType PointT;
            pc_->template inputCloud<PointT>(cloud);
        }

    private:
        T* pc_;
    };

    typedef boost::shared_ptr<PointCloudMessage> Ptr;

    typedef typename boost::make_variant_over<
    typename boost::mpl::transform<
    PointCloudPointTypes, add_point_cloud_ptr<boost::mpl::_1>
    >::type
    >::type variant;

    PointCloudMessage()
        : Message ("PointCloud")
    {

    }

    virtual ConnectionType::Ptr clone() {
        Ptr new_msg(new PointCloudMessage);
        new_msg->value = value;
        return new_msg;
    }

    virtual ConnectionType::Ptr toType() {
        Ptr new_msg(new PointCloudMessage);
        return new_msg;
    }

    static ConnectionType::Ptr make(){
        Ptr new_msg(new PointCloudMessage);
        return new_msg;
    }

    bool acceptsConnectionFrom(const ConnectionType* other_side) const {
        return name() == other_side->name();
    }

    void writeYaml(YAML::Emitter& yaml) {
        yaml << YAML::Key << "value" << YAML::Value << "not implemented";
    }
    void readYaml(YAML::Node& node) {
    }

    variant value;
};

}
}

#endif // POINT_CLOUD_MESSAGE_H
