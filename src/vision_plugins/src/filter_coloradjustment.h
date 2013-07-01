#ifndef FILTER_COLORCHANNEL_H
#define FILTER_COLORCHANNEL_H
#include <vision_evaluator/filter.h>

class QSlider;
class QDoubleSlider;
class QCheckBox;
class QComboBox;

namespace vision_evaluator {
class ColorAdjustment : public vision_evaluator::BoxedObject
{
    Q_OBJECT

public:
    ColorAdjustment();
    virtual ~ColorAdjustment();

    virtual void fill(QBoxLayout *parent);

private Q_SLOTS:
    virtual void messageArrived(ConnectorIn *source);
    void setPreset(QString text);

private:
    /// internal typedefs
    typedef std::vector<std::pair<QDoubleSlider*, QDoubleSlider*> > ChannelLimits;

    /// connectors
    ConnectorIn   *input_;
    ConnectorOut  *output_;

    /// presets
    enum Preset{NONE, HSV, HSL, STD};
    Preset  active_preset_;
    int     channel_count_;

    /// layout and computation
    QBoxLayout                 *layout_;
    QCheckBox                  *check_normalize_;
    QSlider                    *slide_lightness_;
    QWidget                    *container_ch_sliders_;
    ChannelLimits               slider_pairs_;
    std::map<QString, Preset>   presets_;

    /// helpers
    void addLightness(cv::Mat &img);
    void prepareSliderPair(QDoubleSlider *min, QDoubleSlider *max);
    void updateSliders();


    /// MEMENTO
    class State : public Memento {
    public:
        void readYaml(const YAML::Node &node)
        {
            node["ch1_min"] >> ch1_min;
            node["ch1_max"] >> ch1_max;
            node["ch2_min"] >> ch2_min;
            node["ch2_max"] >> ch2_max;
            node["ch3_min"] >> ch3_min;
            node["ch3_max"] >> ch3_max;
        }

        void writeYaml(YAML::Emitter &out) const
        {
            out << YAML::Key << "ch1_min" << YAML::Value << ch1_min;
            out << YAML::Key << "ch1_max" << YAML::Value << ch1_max;
            out << YAML::Key << "ch2_min" << YAML::Value << ch2_min;
            out << YAML::Key << "ch2_max" << YAML::Value << ch2_max;
            out << YAML::Key << "ch3_min" << YAML::Value << ch3_min;
            out << YAML::Key << "ch3_max" << YAML::Value << ch3_max;
        }

    public:
        double ch1_min;
        double ch1_max;
        double ch2_min;
        double ch2_max;
        double ch3_min;
        double ch3_max;
    };


};
}

#endif // FILTER_COLORCHANNEL_H
