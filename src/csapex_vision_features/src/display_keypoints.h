#ifndef DisplayFeatures_H
#define DisplayFeatures_H

/// COMPONENT
#include <csapex/boxed_object.h>

/// SYSTEM
#include <opencv2/opencv.hpp>
#include <QComboBox>
#include <QCheckBox>

namespace csapex
{

class DisplayKeypoints : public csapex::BoxedObject
{
    Q_OBJECT

public:
    DisplayKeypoints();

public:
    virtual void fill(QBoxLayout* layout);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

public Q_SLOTS:
    virtual void messageArrived(ConnectorIn* source);

private Q_SLOTS:
    void update(int slot);
    void update();

    // TODO: state
private:
    ConnectorIn* in_img;
    ConnectorIn* in_key;

    ConnectorOut* out_img;

    QComboBox* colorbox;
    QCheckBox* richbox;

    bool has_img;
    bool has_key;

    struct State : public Memento {
        cv::Scalar color;
        int flags;

        virtual void writeYaml(YAML::Emitter& out) const;
        virtual void readYaml(const YAML::Node& node);
    };

    State state;

};

}

#endif // DisplayFeatures_H
