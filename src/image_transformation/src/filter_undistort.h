#ifndef FILTER_UNDISTORT_H
#define FILTER_UNDISTORT_H

/// COMPONENT
#include <vision_evaluator/filter.h>
#include "undistort.h"

/// SYSTEM
#include <QFileDialog>
#include <QTextEdit>
#include <QPushButton>
#include <QSlider>

namespace image_transformation {
class Undistort : public vision_evaluator::Filter
{
    Q_OBJECT

public:
    Undistort();

    virtual ~Undistort();

    virtual void filter(cv::Mat &img, cv::Mat &mask);
    virtual void insert(QBoxLayout *parent);

public Q_SLOTS:
    void search();

private:
    Undistorter     *undist_;
    QTextEdit       *path_box_;
    QPushButton     *file_dialog_;
    QSlider         *margin_;

    void read_matrices(const std::string &path, cv::Mat &intrinsics, cv::Mat &distortion_coeffs);

};

}

#endif // FILTER_UNDISTORT_H
