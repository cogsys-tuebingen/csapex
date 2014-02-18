/// HEADER
#include "passthrough.h"

/// PROJECT

#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex/utility/qt_helper.hpp>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <boost/mpl/for_each.hpp>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>

CSAPEX_REGISTER_CLASS(csapex::PassThrough, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

PassThrough::PassThrough()
    : min_(0)
{
    addTag(Tag::get("PointCloud"));
}

void PassThrough::fill(QBoxLayout *layout)
{
    setSynchronizedInputs(true);

    input_cloud_ = addInput<PointCloudMessage>("PointCloud");

    output_pos_ = addOutput<PointCloudMessage>("cropped PointCloud (+)");
    output_neg_ = addOutput<PointCloudMessage>("cropped PointCloud (-)");

    min_ = QtHelper::makeDoubleSlider(layout, "min", 0.0, -5.0, 5.0, 0.01);
    max_ = QtHelper::makeDoubleSlider(layout, "max", 0.0, -5.0, 5.0, 0.01);

    QObject::connect(min_, SIGNAL(valueChanged(double)), this, SLOT(update()));
    QObject::connect(max_, SIGNAL(valueChanged(double)), this, SLOT(update()));

    field = new QComboBox;
    layout->addWidget(field);
    QObject::connect(field, SIGNAL(currentIndexChanged(int)), this, SLOT(updateFields()));

    updateFields();

    update();
}

void PassThrough::updateFields(const std::vector<std::string>& fields)
{
    bool rebuild = false;
    for(int i = 0, n = field->count(); i < n; ++i) {
        std::string f = field->itemData(i, Qt::DisplayRole).toString().toStdString();
        if(std::find(fields.begin(), fields.end(), f) == fields.end()) {
            rebuild = true;
            break;
        }
    }
    if(rebuild) {
        field->clear();

        int i = 0;
        BOOST_FOREACH (const std::string& f, fields) {
            field->addItem(f.c_str());
            if(f == state.field) {
                field->setCurrentIndex(i);
            }
            ++i;
        }
    }
}

void PassThrough::updateFields()
{
    if(field->count() == 0) {
        field->addItem("");
    }

    if(field->currentText() != "") {
        state.field = field->currentText().toStdString();
    }

    if(field->currentText().toStdString() != state.field) {
        for(int i = 0, n = field->count(); i < n; ++i) {
            std::string f = field->itemData(i, Qt::DisplayRole).toString().toStdString();
            if(f == state.field) {
                field->setCurrentIndex(i);
                return;
            }
        }

        // field not there
        field->addItem(state.field.c_str());
        field->setCurrentIndex(field->count()-1);
    }
}

void PassThrough::update()
{
    if(!signalsBlocked()) {
        state.min_ = min_->doubleValue();
        state.max_ = max_->doubleValue();
    }
}

void PassThrough::process()
{
    PointCloudMessage::Ptr msg(input_cloud_->getMessage<PointCloudMessage>());

    boost::apply_visitor (PointCloudMessage::Dispatch<PassThrough>(this), msg->value);
}

template <class PointT>
void PassThrough::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::vector<pcl::PCLPointField> fields;
    std::vector<std::string> field_names;
    pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(pcl::detail::FieldAdder<PointT>(fields));

    for(size_t d = 0; d < fields.size (); ++d) {
        field_names.push_back(fields[d].name);
    }

    updateFields(field_names);


    // check available fields!
    pcl::PassThrough<PointT> pass;
    pass.setFilterFieldName (state.field);
    pass.setFilterLimits (state.min_, state.max_);
    pass.setInputCloud(cloud);

    if(output_pos_->isConnected()) {
        typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pass.filter(*out);
        out->header = cloud->header;

        PointCloudMessage::Ptr msg(new PointCloudMessage);
        msg->value = out;
        output_pos_->publish(msg);
    }

    if(output_neg_->isConnected()) {
        typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pass.setNegative(true);
        pass.filter(*out);
        out->header = cloud->header;

        PointCloudMessage::Ptr msg(new PointCloudMessage);
        msg->value = out;
        output_neg_->publish(msg);
    }

}

Memento::Ptr PassThrough::getState() const
{
    return boost::shared_ptr<State>(new State(state));
}

void PassThrough::setState(Memento::Ptr memento)
{
    boost::shared_ptr<PassThrough::State> m = boost::dynamic_pointer_cast<PassThrough::State> (memento);
    assert(m.get());

    state = *m;


    if(min_) {
        blockSignals(true);
        min_->setDoubleValue(state.min_);
        max_->setDoubleValue(state.max_);
        updateFields();
        blockSignals(false);
    }
}

void PassThrough::State::writeYaml(YAML::Emitter& out) const {
    out << YAML::Key << "dim" << YAML::Value << YAML::Flow << YAML::BeginSeq << min_ << max_<< YAML::EndSeq;
    out << YAML::Key << "field" << YAML::Value << field;
}
void PassThrough::State::readYaml(const YAML::Node& node) {
    if(node.FindValue("dim")) {
        const YAML::Node& seq = node["dim"];
        assert(seq.Type() == YAML::NodeType::Sequence);
        seq[0] >> min_;
        seq[1] >> max_;
    }
    if(node.FindValue("field")) {
        node["field"] >> field;
    }
}
