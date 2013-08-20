#ifndef VIRTUAL_CAMERA_H
#define VIRTUAL_CAMERA_H

/// PROJECT
#include <csapex/boxed_object.h>
#include <csapex_vision/image_provider.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/qt_helper.hpp>
#include <utils/LibCvTools/perspective_transform.h>

/// SYSTEM
#include <QPushButton>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace csapex
{

class ConnectorOut;

class VirtualCamera : public BoxedObject
{
    Q_OBJECT

public:
    VirtualCamera();
    ~VirtualCamera();

    virtual void fill(QBoxLayout* layout);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

    void import(const QString& filename);

public Q_SLOTS:
    virtual void messageArrived(ConnectorIn* source);
    void tick();

    void importDialog();
    void toggle(bool on);
    void compute();
    void updatePose();

private:
    bool doImport(const QString& filename);

private:
    struct State : public Memento {
        State(VirtualCamera* parent)
            : parent(parent)
        {}

        VirtualCamera* parent;

        QString last_path_;
        Eigen::Vector3d pos;
        Eigen::Vector3d rot;

        double focal_length;
        int w;
        int h;

        Memento::Ptr sub_state;

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);
    };

    State state;

    ImageProvider::Ptr provider_;

    PerspectiveTransformer transformer_;

    cv::Mat map;
    cv::Mat map_out;
    cv::Mat view;

    connection_types::CvMatMessage::Ptr map_msg;
    connection_types::CvMatMessage::Ptr view_msg;

    ConnectorOut* output_view_;
    ConnectorOut* output_map_;

    QSlider* w;
    QSlider* h;

    QDoubleSlider* focal_length;

    QDoubleSlider* x;
    QDoubleSlider* y;
    QDoubleSlider* z;
    QDoubleSlider* roll;
    QDoubleSlider* pitch;
    QDoubleSlider* yaw;

    QHBoxLayout* additional_layout_;
    QPushButton* file_dialog_;

    bool dirty;
};

}

#endif // VIRTUAL_CAMERA_H
