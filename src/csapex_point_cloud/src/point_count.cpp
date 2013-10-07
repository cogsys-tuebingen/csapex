/// HEADER
#include "point_count.h"

/// PROJECT
#include <csapex/model/box.h>
#include <csapex/model/connector_in.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <boost/mpl/for_each.hpp>

PLUGINLIB_EXPORT_CLASS(csapex::PointCount, csapex::BoxedObject)

using namespace csapex;
using namespace csapex::connection_types;

PointCount::PointCount()
{
    addTag(Tag::get("PointCloud"));
}

void PointCount::fill(QBoxLayout *layout)
{
    box_->setSynchronizedInputs(true);

    input_ = new ConnectorIn(box_, 0);
    input_->setLabel("PointCloud");
    input_->setType(connection_types::PointCloudMessage::make());

    number_ = new QLCDNumber;
    number_->setDigitCount(8);
    layout->addWidget(number_);

    box_->addInput(input_);
}

void PointCount::allConnectorsArrived()
{
    PointCloudMessage::Ptr msg(input_->getMessage<PointCloudMessage>());

    boost::apply_visitor (PointCloudMessage::Dispatch<PointCount>(this), msg->value);
}

template <class PointT>
void PointCount::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    int c = cloud->points.size();
    number_->display(c);
}
