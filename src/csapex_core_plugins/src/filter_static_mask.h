#ifndef FILTER_STATIC_MASK_H
#define FILTER_STATIC_MASK_H

/// PROJECT
#include <csapex_vision/filter.h>

/// SYSTEM
#include <QDialog>
#include <QGraphicsView>
#include <QPushButton>
#include <fstream>

namespace csapex
{

class ModalPainter;

class FilterStaticMask : public Filter
{
    Q_OBJECT

public:
    FilterStaticMask();
    virtual ~FilterStaticMask();

public:
    virtual void insert(QBoxLayout*);

    virtual Memento::Ptr getState() const;
    virtual void setState(Memento::Ptr memento);

public Q_SLOTS:
    virtual void filter(cv::Mat& img, cv::Mat& mask);
    void new_mask(cv::Mat m);
    void showPainter();

Q_SIGNALS:
    void input(cv::Mat);

private:
    ModalPainter* painter;

    QPushButton* btn;

    struct State : public Memento {
        cv::Mat mask_;

        State(FilterStaticMask* parent)
            : parent(parent)
        {}

        virtual void writeYaml(YAML::Emitter& out) const {
            std::string file = parent->getName() + ".ppm"; // TODO: also use random part
            out << YAML::Key << "mask" << YAML::Value << file;
            cv::imwrite(file, mask_);

        }
        virtual void readYaml(const YAML::Node& node) {
            if(!node.FindValue("mask")){
                return;
            }
            std::string file;
            node["mask"] >> file;
            mask_ = cv::imread(file, 0);
        }

        FilterStaticMask* parent;
    };

    State state;
};






class ModalPainter : public QObject
{
    Q_OBJECT

public:
    ModalPainter();
    virtual ~ModalPainter();
    void run();

    bool eventFilter(QObject* obj, QEvent* event);

public Q_SLOTS:
    void input(cv::Mat img);
    void publish_mask();
    void reset_mask();

Q_SIGNALS:
    void new_mask(cv::Mat);

private:
    QDialog* modal;
    QPushButton* reset;
    QPushButton* keep;
    QGraphicsView* view;
    QGraphicsScene* scene;

    cv::Mat mask;
    cv::Mat mask_backup;

    QPointF start_drag;
    Qt::MouseButton start_btn;
    bool dragging;
};


} /// NAMESPACE

#endif // FILTER_STATIC_MASK_H
