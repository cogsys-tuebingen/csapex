#ifndef PASSTHROUGH_H_
#define PASSTHROUGH_H_

/// PROJECT
#include <csapex/model/boxed_object.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex/utility/qdouble_slider.h>

/// SYSTEM
#include <QComboBox>

namespace csapex {

class PassThrough : public BoxedObject
{
    Q_OBJECT

public:
    PassThrough();

    virtual void fill(QBoxLayout* layout);
    virtual void allConnectorsArrived();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

public Q_SLOTS:
    virtual void update();
    void updateFields();
    void updateFields(const std::vector<std::string> &fields);

private:
    ConnectorIn* input_cloud_;
    ConnectorOut* output_pos_;
    ConnectorOut* output_neg_;

    QDoubleSlider* min_;
    QDoubleSlider* max_;

    QComboBox* field;

    struct State : public Memento {
        double min_, max_;
        std::string field;

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);
    };

    State state;
};

}

#endif // PASSTHROUGH_H_
