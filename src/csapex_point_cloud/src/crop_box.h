#ifndef CROP_BOX_H_
#define CROP_BOX_H_

/// PROJECT
#include <csapex/model/boxed_object.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex/utility/qdouble_slider.h>

namespace csapex {

class CropBox : public BoxedObject
{
    Q_OBJECT

public:
    CropBox();

    virtual void fill(QBoxLayout* layout);
    virtual void allConnectorsArrived();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

public Q_SLOTS:
    virtual void update();

private:
    ConnectorIn* input_cloud_;
    ConnectorOut* output_pos_;
    ConnectorOut* output_neg_;

    QDoubleSlider* x_;
    QDoubleSlider* y_;
    QDoubleSlider* z_;

    QDoubleSlider* dx_;
    QDoubleSlider* dy_;
    QDoubleSlider* dz_;

    struct State : public Memento {
        double x_, y_, z_;
        double dx_, dy_, dz_;

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);
    };

    State state;
};

}

#endif // CROP_BOX_H_
