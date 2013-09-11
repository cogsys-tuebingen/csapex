#ifndef OBJECT_DETECTION_H
#define OBJECT_DETECTION_H

/// PROJECT
#include <csapex/model/boxed_object.h>
#include <csapex/csapex_fwd.h>
#include <QSlider>
#include <QComboBox>
#include "object_homography.h"

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>


/// SYSTEM
#include <QPushButton>

namespace csapex
{

class ObjectDetection : public BoxedObject
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<ObjectDetection> Ptr;

public:
    ObjectDetection();

    /// insert GUI elements
    virtual void fill(QBoxLayout* layout);

private Q_SLOTS:
    /// callback when a message arrives
    void messageArrived(ConnectorIn* source);
    virtual void updateDynamicGui(QBoxLayout *layout);
    void updateDetector();


    /// callback from UI button
    void buttonPressed();
    void updateSliders();

private:
    // Presets
    enum Preset{NONE, SURF, DUMMY1, DUMMY2};

    //Methods
    /// connectors that were added to the parent box
    ConnectorIn* in_a_;
    ConnectorIn* in_b_;
    ConnectorIn* in_c_;
    ConnectorIn* in_d_;
    ConnectorOut* out_;

    Preset active_preset_;

    /// Layout
    QSlider                     *hessian_slider_;
    QPushButton                 *btn_;
    QComboBox                   *detectorbox_;
    QComboBox                   *extractorbox_;
    QComboBox                   *matcherbox_;
    QWidget                     *container_sliders_;

    /// flags to remember, whether both images have been received
    bool has_a_;
    bool has_b_;
    bool has_c_;
    bool has_d_;

    /// flag to remember, which image to re-publish
    bool publish_a_;

    /// Value minHessian for SurfDetector
    int minHessian;
};

} // namespace csapex

#endif // OBJECT_DETECTION_H
