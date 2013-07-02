#ifndef VIRTUAL_CAMERA_H
#define VIRTUAL_CAMERA_H

/// PROJECT
#include <designer/boxed_object.h>
#include <vision_evaluator/image_provider.h>
#include <vision_evaluator/messages_default.hpp>
#include <qt_helper.hpp>
#include <utils/LibCvTools/perspective_transform.h>

/// SYSTEM
#include <QPushButton>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace vision_evaluator
{

class ConnectorOut;

class VirtualCamera;

class VirtualCameraWorker : public QObject
{
    Q_OBJECT

    friend class VirtualCamera;
    friend class State;

public:
    VirtualCameraWorker(VirtualCamera* parent);

public Q_SLOTS:
    void publish();
    bool import(const QString& path);

    void updatePose();

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
    QTimer* timer_;

    PerspectiveTransformer transformer_;

    cv::Mat map;

    CvMatMessage::Ptr map_out;
    CvMatMessage::Ptr view;

    ConnectorOut* output_view_;
    ConnectorOut* output_map_;
};

class VirtualCamera : public BoxedObject
{
    Q_OBJECT

    friend class VirtualCameraWorker;
    friend class VirtualCameraWorker::State;

public:
    VirtualCamera();
    ~VirtualCamera();

    virtual void fill(QBoxLayout* layout);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

    void import(const QString& filename);

public Q_SLOTS:
    void importDialog();
    void toggle(bool on);

private:
    VirtualCameraWorker* worker;


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
};

}

#endif // VIRTUAL_CAMERA_H
