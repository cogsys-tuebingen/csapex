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

    void         setState(Memento::Ptr memento);
    Memento::Ptr getState() const;

    virtual void fill(QBoxLayout *parent);

private Q_SLOTS:
    virtual void messageArrived(ConnectorIn *source);
    void setPreset(int index);

private:
    /// presets
    enum Preset{NONE, HSV, HSL, STD};

    /// internal typedefs
    typedef std::vector<std::pair<QDoubleSlider*, QDoubleSlider*> > ChannelLimits;
    typedef std::pair<int, Preset> intPresetPair;
    typedef std::pair<Preset, int> presetIntPair;

    /// connectors
    ConnectorIn   *input_;
    ConnectorOut  *output_;

    Preset  active_preset_;
    int     channel_count_;

    /// layout and computation
    QBoxLayout                 *layout_;
    QCheckBox                  *check_normalize_;
    QSlider                    *slide_lightness_;
    QComboBox                  *combo_preset_;
    QWidget                    *container_ch_sliders_;
    ChannelLimits               slider_pairs_;
    std::map<int, Preset>       index_to_preset_;
    std::map<Preset, int>       preset_to_index_;

    /// helpers
    void addLightness(cv::Mat &img);
    void prepareSliderPair(QDoubleSlider *min, QDoubleSlider *max);
    void updateSliders();

    /// memento
    void updateState();
    class State : public Memento {
    public:
        void readYaml(const YAML::Node &node)
        {
            const YAML::Node &min_values = node["mins"];
            for(YAML::Iterator it = min_values.begin() ; it != min_values.end() ; it++) {
                double min;
                *it >> min;
                mins.push_back(min);
            }

            const YAML::Node &max_values = node["maxs"];
            for(YAML::Iterator it = max_values.begin() ; it != max_values.end() ; it++) {
                double max;
                *it >> max;
                maxs.push_back(max);
            }

            node["channel_count"] >> channel_count;
            node["normalize"] >> normalize;
            node["lightness"] >> lightness;
            int preset_int;
            node["preset"] >> preset_int;
            preset = static_cast<Preset>(preset_int);
        }

        void writeYaml(YAML::Emitter &out) const
        {
            out << YAML::Key << "mins" << YAML::Value << YAML::BeginSeq;
            for(std::vector<double>::const_iterator it = mins.begin() ; it != mins.end() ; it++) {
                out << *it;
            }
            out << YAML::EndSeq;
            out << YAML::Key << "maxs" << YAML::Value << YAML::BeginSeq;
            for(std::vector<double>::const_iterator it = maxs.begin() ; it != maxs.end() ; it++) {
                out << *it;
            }
            out << YAML::EndSeq;
            out << YAML::Key << "channel_count" << YAML::Value << channel_count;
            out << YAML::Key << "normalize" << YAML::Value << normalize;
            out << YAML::Key << "preset" << YAML::Value << (int) preset;
            out << YAML::Key << "lightness" << YAML::Value << lightness;
        }

    public:
        std::vector<double> mins;
        std::vector<double> maxs;
        int     channel_count;
        bool    normalize;
        Preset  preset;
        int     lightness;

    };

    State state_;


};
}

#endif // FILTER_COLORCHANNEL_H
