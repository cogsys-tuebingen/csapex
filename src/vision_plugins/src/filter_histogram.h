#ifndef FILTER_HISTOGRAM_H
#define FILTER_HISTOGRAM_H

class QDoubleSlider;
class QCheckBox;
class QComboBox;

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

private:
    enum Preset{NONE, RGB, BGR, HSV, HSL};
    Preset       active_preset_;

    ConnectorIn   *input_;
    ConnectorOut  *output_;
    ConnectorOut  *output_histogram_;
    QBoxLayout    *layout_;
    QWidget       *slide_ch_container_;
    QWidget       *slide_ch_scale_container_;
    QDoubleSlider *slide_ch_scale_;
    QCheckBox     *check_ch_norm_;
    QComboBox     *combo_ch_norm_;
    QCheckBox     *check_equal_;

    std::map<QString, Preset>   presets_;
    std::vector<QDoubleSlider*> channels_;

    std::vector<cv::Scalar> colors;
};
}
#endif // FILTER_HISTOGRAM_H
