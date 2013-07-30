#ifndef COMBINER_SET_OPERATION_H
#define COMBINER_SET_OPERATION_H

/// COMPONENT
#include <vision_evaluator/image_combiner.h>
class QComboBox;

namespace csapex {
/**
 * @brief The SetOperation class can be used to compare two images using a grid
 *        overlay. The Feature observed in this case is the mean value of values given
 *        in a grid cell.
 */
class SetOperation : public ImageCombiner
{
    Q_OBJECT

protected:
    /// MEMENTO
    class State : public Memento {
    public:
        virtual void readYaml(const YAML::Node &node);
        virtual void writeYaml(YAML::Emitter &out) const;

        typedef boost::shared_ptr<State> Ptr;

    public:
        int operation_index;
    };

public:
    SetOperation();
    /**
     * @brief combine - see base class information
     */
    virtual cv::Mat combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2);

    /**
     * @brief Update the gui state.
     * @param layout    the layout
     */
    virtual void updateDynamicGui(QBoxLayout *layout);


    /// MEMENTO
    void         setState(Memento::Ptr memento);
    Memento::Ptr getState() const;

protected Q_SLOTS:
    virtual void updateState(int i);

protected:
    /// fill with standard gui elements
    virtual void fill(QBoxLayout *layout);

    QComboBox  *combo_compare_;

    State state_;
};
}
#endif // COMBINER_SET_OPERATION_H
