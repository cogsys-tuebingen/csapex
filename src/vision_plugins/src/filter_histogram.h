#ifndef FILTER_HISTOGRAM_H
#define FILTER_HISTOGRAM_H

class QDoubleSlider;
class QCheckBox;
class QComboBox;
class QSlider;

/// COMPONENT
#include <vision_evaluator/filter.h>

namespace vision_evaluator {
class Histogram : public vision_evaluator::BoxedObject
{
    Q_OBJECT

public:
    Histogram();

    virtual void fill(QBoxLayout* layout);

private Q_SLOTS:
    void messageArrived(ConnectorIn* source);
    void selectedPreset(QString text);
    void enableNorm(bool value);
    void enableScale(bool value);

private:
    enum Preset{NONE, RGB, BGR, HSV, HSL};
    Preset       active_preset_;

    ConnectorIn   *input_;
    ConnectorOut  *output_;
    ConnectorOut  *output_histogram_;
    QBoxLayout    *layout_;
    QSlider       *slide_lightness_;
    QDoubleSlider *slide_ch_zoom_;
    QWidget       *slide_ch_container_;
    QWidget       *zoom_container_;
    QCheckBox     *check_zoom_;
    QCheckBox     *check_ch_norm_;
    QComboBox     *combo_ch_norm_;
    QCheckBox     *check_equal_;

    std::map<QString, Preset>   presets_;
    std::vector<QDoubleSlider*> channels_;

    std::vector<cv::Scalar> colors_;

    void addLightness(cv::Mat &img);

};
}
#endif // FILTER_HISTOGRAM_H
