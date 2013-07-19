#ifndef FILTER_DEBAYER_H
#define FILTER_DEBAYER_H

/// COMPONENT
#include <vision_evaluator/filter.h>

class QComboBox;

namespace vision_plugins {
/**
 * @brief The Debayer class can be used to debayer raw images to get color images.
 */
class Debayer : public vision_evaluator::Filter
{
    Q_OBJECT

public:
    /**
     * @brief Debayer default constructor.
     */
    Debayer();
    /**
     * @brief ~Debayer destructor.
     */
    virtual ~Debayer();
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
    typedef std::pair<int, int> modePair;

    std::map<int, int> modeFromCombo;

    QComboBox *combo_mode_;

    virtual bool usesMask();
    void fillCombo(QComboBox *combo, std::map<int, int> &map);

    /// MEMENTO
    class State : public Memento {
    public:
        void readYaml(const YAML::Node &node)
        {
            node["input_index"] >> index;
        }

        void writeYaml(YAML::Emitter &out) const
        {
            out << YAML::Key << "input_index" << YAML::Value << index;
        }

    public:
        int     index;
    };

    State state_;
};

}
#endif // FILTER_DEBAYER_H
