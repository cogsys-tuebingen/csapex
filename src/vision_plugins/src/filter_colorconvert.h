#ifndef FILTER_COLORCONVERT_H
#define FILTER_COLORCONVERT_H

/// COMPONENT
#include <csapex_vision/filter.h>

class QComboBox;

namespace vision_plugins {
/**
 * @brief The ColorConvert class can be used to convert colored images from one color space
 *        to another.
 */
class ColorConvert : public csapex::Filter
{
    Q_OBJECT

public:
    /**
     * @brief ColorConvert default constructor.
     */
    ColorConvert();
    /**
     * @brief ~ColorConvert destructor.
     */
    virtual ~ColorConvert();
    /**
     * @brief See base class definition.
     */
    virtual void insert(QBoxLayout *parent);
    /**
     * @brief See base class definition.
     */
    virtual void filter(cv::Mat &img, cv::Mat &mask);

    /// MEMENTO
    void         setState(Memento::Ptr memento);
    Memento::Ptr getState() const;

protected:
    /// internal typdefs
    enum ColorSpace {YUV, RGB, BGR, HSL, HSV};
    typedef std::pair<ColorSpace, ColorSpace> csPair;
    typedef std::pair<csPair, int>     csiPair;
    typedef std::pair<int, ColorSpace> icsPair;

    std::map<csPair, int> cs_pair_to_operation_;
    std::map<int, ColorSpace> index_to_cs_in_;
    std::map<int, ColorSpace> index_to_cs_out_;

    QComboBox *combo_in_;
    QComboBox *combo_out_;


    virtual bool usesMask();
    void fillCombo(QComboBox *combo, std::map<int, ColorSpace> &map);

    /// MEMENTO
    class State : public Memento {
    public:
        void readYaml(const YAML::Node &node)
        {
            node["input_index"] >> input_index;
            node["output_index"] >> output_index;
        }

        void writeYaml(YAML::Emitter &out) const
        {
            out << YAML::Key << "input_index" << YAML::Value << input_index;
            out << YAML::Key << "output_index" << YAML::Value << output_index;
        }

    public:
        int     input_index;
        int     output_index;
    };

    State state_;
};

}
#endif // FILTER_COLORCONVERT_H
