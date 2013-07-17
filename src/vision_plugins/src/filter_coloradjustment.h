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

    /// helpers
    void addLightness(cv::Mat &img);
    void prepareSliderPair(QDoubleSlider *min, QDoubleSlider *max);
    void updateSliders();

    /// MEMENTO
    void updateState();
    class State : public Memento {
    public:
        void readYaml(const YAML::Node &node);
        void writeYaml(YAML::Emitter &out) const;

    public:
        std::vector<double> mins;
        std::vector<double> maxs;
        int     channel_count;
        bool    normalize;
        int     lightness;
        int     combo_index;

    };

    State state_;


};
}

#endif // FILTER_COLORCHANNEL_H
