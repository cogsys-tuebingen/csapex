/// HEADER
#include "crop_box.h"

/// PROJECT
#include <csapex/model/box.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex/utility/qt_helper.hpp>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <boost/mpl/for_each.hpp>
#include <pcl_ros/transforms.h>
#include <pcl/filters/crop_box.h>

CSAPEX_REGISTER_CLASS(csapex::CropBox, csapex::BoxedObject)

using namespace csapex;
using namespace csapex::connection_types;

CropBox::CropBox()
{
    addTag(Tag::get("PointCloud"));
}

void CropBox::fill(QBoxLayout *layout)
{
    setSynchronizedInputs(true);

    input_cloud_ = addInput<PointCloudMessage>("PointCloud");

    output_pos_ = addOutput<PointCloudMessage>("cropped PointCloud (+)");
    output_neg_ = addOutput<PointCloudMessage>("cropped PointCloud (-)");

    x_ = QtHelper::makeDoubleSlider(layout, "origin/x", 0.0, -5.0, 5.0, 0.01);
    y_ = QtHelper::makeDoubleSlider(layout, "origin/y", 0.0, -5.0, 5.0, 0.01);
    z_ = QtHelper::makeDoubleSlider(layout, "origin/z", 0.0, -5.0, 5.0, 0.01);

    dy_ = QtHelper::makeDoubleSlider(layout, "dimension/y", 1.0, 0.0, 20.0, 0.01);
    dx_ = QtHelper::makeDoubleSlider(layout, "dimension/x", 1.0, 0.0, 20.0, 0.01);
    dz_ = QtHelper::makeDoubleSlider(layout, "dimension/z", 1.0, 0.0, 20.0, 0.01);

    QObject::connect(x_, SIGNAL(valueChanged(double)), this, SLOT(update()));
    QObject::connect(y_, SIGNAL(valueChanged(double)), this, SLOT(update()));
    QObject::connect(z_, SIGNAL(valueChanged(double)), this, SLOT(update()));
    QObject::connect(dx_, SIGNAL(valueChanged(double)), this, SLOT(update()));
    QObject::connect(dy_, SIGNAL(valueChanged(double)), this, SLOT(update()));
    QObject::connect(dz_, SIGNAL(valueChanged(double)), this, SLOT(update()));

    update();
}

void CropBox::update()
{
    if(!signalsBlocked()) {
        state.x_ = x_->doubleValue();
        state.y_ = y_->doubleValue();
        state.z_ = z_->doubleValue();
        state.dx_ = dx_->doubleValue();
        state.dy_ = dy_->doubleValue();
        state.dz_ = dz_->doubleValue();
    }
}

void CropBox::allConnectorsArrived()
{
    PointCloudMessage::Ptr msg(input_cloud_->getMessage<PointCloudMessage>());

    boost::apply_visitor (PointCloudMessage::Dispatch<CropBox>(this), msg->value);
}

template <class PointT>
void CropBox::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    Eigen::Vector4f o(state.x_, state.y_, state.z_, 0);
    Eigen::Vector4f dim(state.dx_, state.dy_, state.dz_, 0);

    Eigen::Vector4f min_pt_ = o - dim/2;
    Eigen::Vector4f max_pt_ = min_pt_ + dim;


    pcl::CropBox<PointT> crop;
    crop.setMin(min_pt_);
    crop.setMax(max_pt_);
    crop.setInputCloud(cloud);

    if(output_pos_->isConnected()) {
        typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        crop.filter(*out);
        out->header = cloud->header;

        PointCloudMessage::Ptr msg(new PointCloudMessage);
        msg->value = out;
        output_pos_->publish(msg);
    }

    if(output_neg_->isConnected()) {
        typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        crop.setNegative(true);
        crop.filter(*out);
        out->header = cloud->header;

        PointCloudMessage::Ptr msg(new PointCloudMessage);
        msg->value = out;
        output_neg_->publish(msg);
    }

}

Memento::Ptr CropBox::getState() const
{
    return boost::shared_ptr<State>(new State(state));
}

void CropBox::setState(Memento::Ptr memento)
{
    boost::shared_ptr<CropBox::State> m = boost::dynamic_pointer_cast<CropBox::State> (memento);
    assert(m.get());

    state = *m;

    blockSignals(true);

    x_->setDoubleValue(state.x_);
    y_->setDoubleValue(state.y_);
    z_->setDoubleValue(state.z_);
    dx_->setDoubleValue(state.dx_);
    dy_->setDoubleValue(state.dy_);
    dz_->setDoubleValue(state.dz_);

    blockSignals(false);
}

void CropBox::State::writeYaml(YAML::Emitter& out) const {
    out << YAML::Key << "dim" << YAML::Value << YAML::Flow << YAML::BeginSeq << x_ << y_ << z_ << dx_ << dy_ << dz_ << YAML::EndSeq;
}
void CropBox::State::readYaml(const YAML::Node& node) {
    if(node.FindValue("dim")) {
        const YAML::Node& seq = node["dim"];
        assert(seq.Type() == YAML::NodeType::Sequence);
        seq[0] >> x_;
        seq[1] >> y_;
        seq[2] >> z_;
        seq[3] >> dx_;
        seq[4] >> dy_;
        seq[5] >> dz_;
    }
}
