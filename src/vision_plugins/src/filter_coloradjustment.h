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
    typedef std::vector<std::pair<QDoubleSlider*, QDoubleSlider*> > ChannelLimits;
    ConnectorIn   *input_;
    ConnectorOut  *output_;

    enum Preset{NONE, CH1, CH2, HSV, HSL, CH3, CH4};
    Preset                      active_preset_;
    int                     channel_count(Preset p);


    QBoxLayout                 *layout_;
    QCheckBox                  *limit_;
    QSlider                    *slide_lightness_;
    QWidget                    *slide_ch_container_;
    QDoubleSlider              *last_updater_;
    std::map<QString, Preset>   presets_;
    ChannelLimits               channel_limits_;


    void addLightness(cv::Mat &img);
    void connectPair(const std::pair<QDoubleSlider *, QDoubleSlider *> &pair);
    void insertPair(QDoubleSlider *min, QDoubleSlider *max);

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
