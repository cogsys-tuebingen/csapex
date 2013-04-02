/*
 * histogram_viewer_widget.h
 *
 *  Created on: 3 14, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef HISTOGRAM_VIEWER_WIDGET_H
#define HISTOGRAM_VIEWER_WIDGET_H

/// SYSTEM
#include <opencv2/opencv.hpp>

#include <QtOpenGL/QGLWidget>
#include <QVector3D>
#include <QVector2D>
#include <QTimer>

class HistogramViewerWidget : public QGLWidget
{
    Q_OBJECT

public:
    HistogramViewerWidget(QWidget* parent = NULL);

    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();

    void mousePressEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
    void wheelEvent(QWheelEvent* event);
    void keyEvent(QKeyEvent* event);

    void setHistogram(cv::Mat& histogram);

    void waitForPainting(int w, int h);

    long drawWithTheta();
    long drawWithoutTheta();
public Q_SLOTS:
    void tick();

private:
    void updateEye();

private:
//    QTimer* repainter;

    GLint w;
    GLint h;

    QVector3D lookat;
    QVector3D eye;
    QVector3D up;

    QVector3D delta;

    QVector2D lastPos;

    double phi;
    double theta;

    double r;

    bool has_list;
    GLuint list;

    cv::Mat histogram_;
};

#endif // HISTOGRAM_VIEWER_WIDGET_H
