#ifndef VOXEL_GRID_H
#define VOXEL_GRID_H

/// PROJECT
#include <csapex/model/boxed_object.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex/utility/qdouble_slider.h>

namespace csapex {

class VoxelGrid : public BoxedObject
{
    Q_OBJECT

public:
    VoxelGrid();

    virtual void fill(QBoxLayout* layout);
    virtual void process();

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

public Q_SLOTS:
    virtual void update();

private:
    ConnectorIn* input_cloud_;
    ConnectorOut* output_;

    QDoubleSlider* res_;

    struct State : public Memento {
        double resolution_;

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);
    };

    State state;
};

}

#endif // VOXEL_GRID_H
