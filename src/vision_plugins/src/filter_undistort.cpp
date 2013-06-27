/// HEADER
#include "filter_undistort.h"

/// COMPONENT
#include <vision_evaluator/qt_helper.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(vision_plugins::Undistort, vision_evaluator::Filter)

using namespace vision_plugins;

Undistort::Undistort() :
    undist_(NULL)
{
}

Undistort::~Undistort()
{
    delete undist_;
}

void Undistort::filter(cv::Mat &img, cv::Mat &mask)
{
    if(undist_ != NULL) {
        int margin = margin_->value();
        cv::Mat copy;

        if(margin != 0 || margin != 0) {
            copy = cv::Mat(img.rows + 2 * margin, img.cols + 2 * margin, img.type());
            cv::Mat roi(copy, cv::Rect(margin, margin, img.cols, img.rows));
            img.copyTo(roi);
        } else {
            copy = img;
        }

        undist_->reset_map(cv::Size(copy.cols, copy.rows), margin, margin);
        undist_->undistort(copy, img);
    }
}

void Undistort::insert(QBoxLayout *parent)
{
    QHBoxLayout *internal_layout = new QHBoxLayout();
    path_box_    = new QTextEdit();
    file_dialog_ = new QPushButton("select");

    path_box_->setReadOnly(true);
    path_box_->setText("Enter a path ...");
    path_box_->setWordWrapMode(QTextOption::NoWrap);
    path_box_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    path_box_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    path_box_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    path_box_->setFixedHeight(path_box_->fontMetrics().height() * 2);


    internal_layout->addWidget(new QLabel("path: "));
    internal_layout->addWidget(path_box_);
    internal_layout->addWidget(file_dialog_);
    parent->addLayout(internal_layout);

    margin_ = QtHelper::makeSlider(parent, "margin", 0, 0, 1000);

    QObject::connect(file_dialog_, SIGNAL(pressed()), this, SLOT(search()));
}

void Undistort::search()
{
    /// fix : maybe use last directory !

    QString filename = QFileDialog::getOpenFileName(
                0,
                tr("Open Camera Parametres"),
                QDir::currentPath(),
                tr("Camera Parametres (*.yaml);;All files (*.*)") );

    if(!filename.isNull()) {
        path_box_->setText(filename);
        cv::Mat intr;
        cv::Mat dist;
        read_matrices(filename.toUtf8().data(), intr, dist);

        if(!intr.empty() && !dist.empty())
            undist_ = new Undistorter(intr, dist);


    } else {
        path_box_->setText("No file selected!");
    }

}

void Undistort::read_matrices(const std::string &path, cv::Mat &intrinsics, cv::Mat &distortion_coeffs)
{
    try {
        cv::FileStorage fs(path, cv::FileStorage::READ);

        if(!fs.isOpened()) {
            path_box_->setText("Couldn't open file!");
        } else {

            try {
                fs["intrinsics"] >> intrinsics;
            } catch(cv::Exception e) {
                path_box_->setText("Couldn't read 'intrinsics' of parameter file!");
            }
            try {
                fs["distortion"] >> distortion_coeffs;
            }      catch(cv::Exception e) {
                path_box_->setText("Couldn't read 'intrinsics' of parameter file!");
            }
        }
    } catch(cv::Exception e) {
        path_box_->setText("Couldn't read file - mal formed!");
    }
}
