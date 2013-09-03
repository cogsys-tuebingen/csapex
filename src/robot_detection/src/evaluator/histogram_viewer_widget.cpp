/*
 * histogram_viewer_widget.cpp
 *
 *  Created on: Mar 14, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// HEADER
#include "histogram_viewer_widget.h"
#include <utils/hough_peak.h>

/// SYSTEM
#include <QMatrix4x4>
#include <GL/glu.h>
#include <cmath>
#include <iostream>
#include <QMouseEvent>
#include <utils/LibUtil/MathHelper.h>


HistogramViewerWidget::HistogramViewerWidget(QWidget* /*parent*/)
    : phi(0), theta(0.1), r(25), has_list(false)
{
    //    repainter = new QTimer();
    //    repainter->setInterval(1000.0 / 30.0);
    //    repainter->start();
    //    QObject::connect(repainter, SIGNAL(timeout()), this, SLOT(tick()));


    up.setX(0);
    up.setY(0);
    up.setZ(1);

    lookat.setX(0);
    lookat.setY(0);
    lookat.setZ(0);

    up.normalize();

    updateEye();
}

void HistogramViewerWidget::setHistogram(cv::Mat& histogram)
{
    histogram.copyTo(histogram_);
    has_list = false;
}

void HistogramViewerWidget::mousePressEvent(QMouseEvent* event)
{
    if(event->buttons()) {
        lastPos.setX(event->x());
        lastPos.setY(event->y());

        event->accept();
        update();
    }
}
void HistogramViewerWidget::mouseMoveEvent(QMouseEvent* event)
{
    if(event->buttons() & (Qt::LeftButton | Qt::RightButton)) {
        int dx = event->x() - lastPos.x();
        int dy = event->y() - lastPos.y();

        lastPos.setX(event->x());
        lastPos.setY(event->y());

        if(event->buttons() & Qt::LeftButton) {
            phi = MathHelper::AngleClamp(phi - dx * 0.01);
            theta = std::min(M_PI, std::max(-M_PI, theta - dy * 0.01));

        } else {
            delta -= up.normalized() * 0.15 * dy;
        }

        updateEye();

        event->accept();
        update();
    }
}

void HistogramViewerWidget::wheelEvent(QWheelEvent* event)
{
    r = std::max(0.1, std::min(100.0, r - event->delta() * 0.01));
    updateEye();
}

void HistogramViewerWidget::keyEvent(QKeyEvent* event)
{
    QVector3D look = (lookat - eye).normalized();
    QVector3D right = QVector3D::crossProduct(look, up).normalized();
    QVector3D forward =  QVector3D::crossProduct(up, right).normalized();

    switch(event->key()) {
    case Qt::Key_W:
        delta += forward * 0.5;
        break;
    case Qt::Key_S:
        delta -= forward * 0.5;
        break;
    case Qt::Key_D:
        delta += right * 0.5;
        break;
    case Qt::Key_A:
        delta -= right * 0.5;
        break;
    }
}

void HistogramViewerWidget::updateEye()
{
    eye.setX(std::cos(phi) * r * std::sin(theta));
    eye.setY(std::sin(phi) * r * std::sin(theta));
    eye.setZ(r * std::cos(theta));
}


void HistogramViewerWidget::tick()
{
    //    Q_EMIT repaint();
}

void HistogramViewerWidget::waitForPainting(int w, int h)
{
    initializeGL();
    resizeGL(w, h);
    paintGL();
}

void HistogramViewerWidget::initializeGL()
{
    glEnable(GL_DEPTH_TEST);

    glEnable(GL_LINE_SMOOTH);

    glShadeModel(GL_SMOOTH);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
}

void HistogramViewerWidget::resizeGL(int w, int h)
{
    this->w = w;
    this->h = h;

    resize(w, h);

    glViewport(0, 0, (GLint)w, (GLint)h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    gluPerspective(45.0f,(GLfloat)w/(GLfloat)h,0.01f,300.0f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

long HistogramViewerWidget::drawWithTheta()
{
    long count = 0;
    long i = 0;
    int* raw_data = (int*) histogram_.data;
    int dim_tx = histogram_.size[HoughData::INDEX_X];
    int dim_ty = histogram_.size[HoughData::INDEX_Y];
    int dim_theta = histogram_.size[HoughData::INDEX_THETA];
    int dim_sigma = histogram_.size[HoughData::INDEX_SIGMA];

    for(int tx = 0; tx < dim_tx; ++tx) {
        for(int ty = 0; ty < dim_ty; ++ty) {
            for(int ts = 0; ts < dim_sigma; ++ts) {
                for(int tt = 0; tt < dim_theta; ++tt) {
                    int magnitude = raw_data[i];

                    if(magnitude > 0) {
                        double v = (magnitude / 5.0);
                        glColor3d(v, v, v);
                        glVertex3d(tx, ty, tt);
                        count++;
                    }
                    i++;
                }
            }
        }
    }

    return count;
}

long HistogramViewerWidget::drawWithoutTheta()
{
    long count = 0;
    long i = 0;
    int* raw_data = (int*) histogram_.data;
    int dim_tx = histogram_.size[HoughData::INDEX_X];
    int dim_ty = histogram_.size[HoughData::INDEX_Y];
    int dim_sigma = histogram_.size[HoughData::INDEX_SIGMA];

    for(int tx = 0; tx < dim_tx; ++tx) {
        for(int ty = 0; ty < dim_ty; ++ty) {
            for(int ts = 0; ts < dim_sigma; ++ts) {
                int magnitude = raw_data[i];

                if(magnitude > 0) {
                    double v = (magnitude / 5.0);
                    glColor3d(v, v, v);
                    glVertex3d(tx, ty, ts);
                    count++;
                }
                i++;
            }
        }
    }

    return count;
}

void HistogramViewerWidget::paintGL()
{
    glClearColor(0.8, 0.8, 1.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    gluLookAt(eye.x(), eye.y(), eye.z(),
              lookat.x(), lookat.y(), lookat.z(),
              up.x(), up.y(), up.z());
    glTranslated(-delta.x(), -delta.y(), -delta.z());

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    glBegin(GL_QUADS);

    glColor3d(1.0, 0.0, 0.0);

    for(int y = -10; y <= 10; ++y) {
        for(int x = -10; x <= 10; ++x) {
            glVertex3d(x, y, 0);
            glVertex3d(x + 1, y, 0);
            glVertex3d(x + 1, y + 1, 0);
            glVertex3d(x, y + 1, 0);
        }
    }

    glEnd();

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);




    if(has_list) {
        glCallList(list);
    } else {
        list = glGenLists(1);
        has_list = true;

        glNewList(list, GL_COMPILE_AND_EXECUTE);

        int dim_tx = histogram_.size[HoughData::INDEX_X];
        int dim_ty = histogram_.size[HoughData::INDEX_Y];

        //        glPointSize(20.0);
        glPointSize(4.0);
        int count = 0;

        double scale = (20.0 / dim_tx);
        glScaled(scale, scale, scale);

        glTranslated(-dim_tx/2, -dim_ty/2, 0);

        glEnable(GL_BLEND);
        //        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glBegin(GL_POINTS);

        if(histogram_.dims == 4) {
            count = drawWithTheta();
        } else {
            count = drawWithoutTheta();
        }

        glEnd();
        glDisable(GL_BLEND);

        glEndList();

        std::cout << "vertex count=" << count << std::endl;
    }
}
