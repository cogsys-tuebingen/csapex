#ifndef FILTER_STATIC_MASK_H
#define FILTER_STATIC_MASK_H

/// COMPONENT
#include "filter.h"

/// SYSTEM
#include <QDialog>
#include <QGraphicsView>
#include <QPushButton>

namespace vision_evaluator
{

class ModalPainter;

class FilterStaticMask : public Filter
{
    Q_OBJECT

public:
    FilterStaticMask();
    virtual ~FilterStaticMask();

public:
    virtual void insert(QLayout*);

public Q_SLOTS:
    virtual void filter(cv::Mat img, cv::Mat mask);
    void new_mask(cv::Mat m);
    void showPainter();

Q_SIGNALS:
    void input(cv::Mat);

private:
    ModalPainter* painter;
    cv::Mat mask_;

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
