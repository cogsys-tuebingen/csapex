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

class DisplayFeatures : public csapex::BoxedObject
{
    Q_OBJECT

public:
    DisplayFeatures();

public:
    virtual void fill(QBoxLayout* layout);

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

    cv::Scalar color;
    int flags;

    QComboBox* colorbox;
    QCheckBox* richbox;

    bool has_img;
    bool has_key;
};

}

#endif // DisplayFeatures_H
