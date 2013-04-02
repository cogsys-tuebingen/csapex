#ifndef PANEL_H
#define PANEL_H

/// SYSTEM
#include <opencv2/opencv.hpp>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QWidget>
#include <QThread>

namespace vision_evaluator
{

class Panel : public QWidget
{
    Q_OBJECT

protected:
    Panel(QWidget* parent = 0);
    virtual ~Panel();

Q_SIGNALS:
    void display_request_gui(const QSharedPointer<QImage>);
    void outputMat(cv::Mat, cv::Mat);

protected Q_SLOTS:
    void display(const QSharedPointer<QImage> qimg);
    void display_request(const QSharedPointer<QImage> image);

protected:
    QGraphicsView* view;
    QGraphicsScene* scene;

    QThread* worker;
};

} /// NAMESPACE

#endif // PANEL_H
