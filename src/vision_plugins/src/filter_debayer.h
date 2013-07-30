#ifndef FILTER_DEBAYER_H
#define FILTER_DEBAYER_H

/// COMPONENT
#include <vision_evaluator/filter.h>

/// SYSTEM
class QComboBox;

namespace vision_plugins {
/**
 * @brief The Debayer class can be used to debayer raw images to get color images.
 */
class Debayer : public vision_evaluator::Filter
{
    Q_OBJECT

public:
    Debayer();

    /**
     * @brief See base class definition.
     */
    virtual void insert(QBoxLayout *parent);
    /**
     * @brief See base class definition.
     */
    virtual void filter(cv::Mat &img, cv::Mat &mask);

    /// MEMENTO
    void setState(Memento::Ptr memento);
    Memento::Ptr getState() const;

private:
    typedef std::pair<int, int> modePair;

    std::map<int, int> modeFromCombo;

    QComboBox *combo_mode_;

    virtual bool usesMask();
    void fillCombo(QComboBox *combo, std::map<int, int> &map);
    void debayerAndResize(cv::Mat& source, cv::Mat& dest);

    /// MEMENTO
    class State : public Memento {
        void readYaml(const YAML::Node &node);
        void writeYaml(YAML::Emitter &out) const;

    public:
        int index;
    };

    State state_;
};

}
#endif // FILTER_DEBAYER_H
