/// HEADER
#include "point_count.h"

/// PROJECT

#include <csapex/model/connector_in.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <boost/mpl/for_each.hpp>

CSAPEX_REGISTER_CLASS(csapex::PointCount, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

PointCount::PointCount()
{
    addTag(Tag::get("PointCloud"));
}

void PointCount::fill(QBoxLayout *layout)
{
    setSynchronizedInputs(true);

    input_ = addInput<PointCloudMessage>("PointCloud");

    number_ = new QLCDNumber;
    number_->setDigitCount(8);
    layout->addWidget(number_);

}

void PointCount::process()
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
