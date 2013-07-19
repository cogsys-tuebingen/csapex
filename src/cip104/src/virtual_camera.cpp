/// HEADER
#include "virtual_camera.h"

/// PROJECT
#include <designer/box.h>
#include <designer/connector_out.h>
#include <qt_helper.hpp>

/// SYSTEM
#include <boost/foreach.hpp>
#include <pluginlib/class_list_macros.h>
#include <QLabel>
#include <QFileDialog>
#include <QTimer>
#include <QtConcurrentRun>

PLUGINLIB_EXPORT_CLASS(vision_evaluator::VirtualCamera, vision_evaluator::BoxedObject)

using namespace vision_evaluator;
using namespace connection_types;

VirtualCameraWorker::VirtualCameraWorker(VirtualCamera *parent)
    : state(parent), timer_(NULL), map_msg(new CvMatMessage), view_msg(new CvMatMessage), output_view_(NULL), output_map_(NULL)
{
    timer_ = new QTimer();
    timer_->setInterval(100);
    timer_->start();

    state.last_path_ = QDir::currentPath();

    QObject::connect(timer_, SIGNAL(timeout()), this, SLOT(publish()));
}

BoxedObject* VirtualCameraWorker::getParent()
{
    return state.parent;
}

void VirtualCameraWorker::State::writeYaml(YAML::Emitter& out) const {
    out << YAML::Key << "path" << YAML::Value << last_path_.toUtf8().constData();
    out << YAML::Key << "focal_length" << YAML::Value << focal_length;
    out << YAML::Key << "size" << YAML::Value <<  YAML::BeginSeq << w << h << YAML::EndSeq;
    out << YAML::Key << "pos" << YAML::Value << YAML::BeginSeq << pos[0] << pos[1] << pos[2] << YAML::EndSeq;
    out << YAML::Key << "rot" << YAML::Value << YAML::BeginSeq << rot[0] << rot[1] << rot[2] << YAML::EndSeq;

    if(parent->worker->provider_->getState().get()) {
        out << YAML::Key << "sub_state";
        out << YAML::Value << YAML::BeginMap;
        parent->worker->provider_->getState()->writeYaml(out);
        out << YAML::EndMap;
    }
}
void VirtualCameraWorker::State::readYaml(const YAML::Node& node) {
    std::string path;
    node["path"] >> path;
    std::cout << "read path: " << path << std::endl;

    last_path_ = QString::fromUtf8(path.c_str());
    parent->import(last_path_);

    if(node.FindValue("sub_state")) {
        const YAML::Node& sub_state_node = node["sub_state"];
        sub_state = parent->worker->provider_->getState();
        sub_state->readYaml(sub_state_node);
    }

    if(node.FindValue("pos")) {
        for(unsigned i = 0; i < 3; ++i) {
            node["pos"][i] >> pos[i];
        }
    }

    if(node.FindValue("rot")) {
        double v[4];
        for(unsigned i = 0; i < 3; ++i) {
            node["rot"][i] >> rot[i];
        }
    }

    if(node.FindValue("size")) {
        node["size"][0] >> w;
        node["size"][1] >> h;
    }

    if(node.FindValue("focal_length")) {
        node["focal_length"] >> focal_length;
    }
}

void VirtualCameraWorker::updatePose()
{
    state.pos[0] = state.parent->x->doubleValue();
    state.pos[1] = state.parent->y->doubleValue();
    state.pos[2] = state.parent->z->doubleValue();

    state.rot[0] = state.parent->pitch->doubleValue();
    state.rot[1] = state.parent->roll->doubleValue();
    state.rot[2] = state.parent->yaw->doubleValue();

    state.w = state.parent->w->value();
    state.h = state.parent->h->value();
    state.focal_length = state.parent->focal_length->doubleValue();



    cv::Point c(map.cols / 2, map.rows / 2);
    cv::Point p = c + cv::Point(state.pos[0], state.pos[1]);


    double a0 = state.rot[0];
    double a1 = state.rot[1];
    double a2 = state.rot[2];
    cv::Mat  rot_x   = ( cv::Mat_<double>(4,4) <<
                         1, 0, 0, 0,
                         0, std::cos(a0), -std::sin(a0), 0,
                         0, std::sin(a0), std::cos(a0), 0,
                         0, 0, 0, 1);
    cv::Mat  rot_y   = ( cv::Mat_<double>(4,4) <<
                         std::cos(a1), 0, std::sin(a1), 0,
                         0, 1, 0, 0,
                         -std::sin(a1), 0, std::cos(a1), 0,
                         0, 0, 0, 1);
    cv::Mat  rot_z   = ( cv::Mat_<double>(4,4) <<
                         std::cos(a2), -std::sin(a2), 0, 0,
                         std::sin(a2), std::cos(a2), 0, 0,
                         0, 0, 1, 0,
                         0, 0, 0, 1);

    if(output_map_->isConnected()) {
        map.copyTo(map_out);
        cv::circle(map_out, p, 1, cv::Scalar(0xFF, 0x00, 0x00), 3, CV_AA);
    }

    double fov = M_PI / 8;

    double w_fov = fov;
    double h_fov = fov;

    double w_view = std::tan(w_fov / 2.0) * 2.0 / (state.focal_length / 1000.0);
    double h_view = std::tan(h_fov / 2.0) * 2.0 / (state.focal_length / 1000.0);

    view.create(state.h, state.w, map.type());

    cv::Mat down = (cv::Mat_<double>(4,1) << 0,0,1.0,1);
    cv::Mat rot = rot_x * rot_y * rot_z;

    uchar* data = view.data;

    int ox = map.cols/2 + state.pos[0];
    int oy = map.rows/2 + state.pos[1];

    double w = state.w;
    double h = state.h;

    for(int y = 0; y < state.h; ++y) {
        for(int x = 0; x < state.w; ++x) {
            double u = -0.5 + x / w;
            double uview = w_view * u;

            double v = -0.5 + y / h;
            double vview = h_view * v;

            cv::Mat dir = (cv::Mat_<double>(4,1) << uview, vview, state.focal_length, 1);
            dir = rot * dir;

            double* dir_data = (double*) dir.data;

            if(dir.dot(down) < 0) {
                *data = 0; ++data;
                *data = 0; ++data;
                *data = 0;

            } else {
                double scale = state.pos[2] / dir_data[2];

                double sx = scale * dir_data[0];
                double sy = scale * dir_data[1];

                int mapx = ox + sx;
                int mapy = oy + sy;

                if(mapy < 0 || mapy >= map.rows || mapx < 0 || mapx >= map.cols){
                    *data = 55; ++data;
                    *data = 55; ++data;
                    *data = 55;
                } else {
                    cv::Vec3b val = map.at<cv::Vec3b>(mapy, mapx);
                    *data = val[0]; ++data;
                    *data = val[1]; ++data;
                    *data = val[2];
                }
            }

            ++data;
        }
    }
}

void VirtualCameraWorker::publish()
{
    if(provider_.get()) {
        view_msg = CvMatMessage::Ptr(new CvMatMessage);
        map_msg = CvMatMessage::Ptr(new CvMatMessage);

        view.copyTo(view_msg->value);
        map_out.copyTo(map_msg->value);

        output_view_->publish(view_msg);
        output_map_->publish(map_msg);
    }
}

bool VirtualCameraWorker::import(const QString& path)
{
    state.last_path_ = path;
    provider_ = ImageProvider::Ptr(ImageProvider::create(state.last_path_.toUtf8().constData()));

    if(!provider_.get()) {
        return false;
    }

    cv::Mat mask;
    provider_->next(map, mask);

    return true;
}


VirtualCamera::VirtualCamera()
    : worker(NULL)
{
}

VirtualCamera::~VirtualCamera()
{
    if(worker != NULL) {
        delete worker;
    }
}

void VirtualCamera::fill(QBoxLayout* layout)
{
    if(worker == NULL) {
        worker = new VirtualCameraWorker(this);

        file_dialog_ = new QPushButton("Import");

        QVBoxLayout* nested = new QVBoxLayout;
        nested->addWidget(file_dialog_);

        additional_layout_ = new QHBoxLayout;
        nested->addLayout(additional_layout_);

        layout->addLayout(nested);

        connect(file_dialog_, SIGNAL(pressed()), this, SLOT(importDialog()));

        worker->output_view_ = new ConnectorOut(box_, 0);
        box_->addOutput(worker->output_view_);

        worker->output_map_ = new ConnectorOut(box_, 1);
        box_->addOutput(worker->output_map_);

        QObject::connect(box_, SIGNAL(toggled(bool)), this, SLOT(toggle(bool)));

        x = QtHelper::makeDoubleSlider(layout, "x", 0.0, -50.0, 50.0, 0.1);
        y = QtHelper::makeDoubleSlider(layout, "y", 0.0, -50.0, 50.0, 0.1);
        z = QtHelper::makeDoubleSlider(layout, "z", 0.0, 0.0, 50.0, 0.1);
        roll = QtHelper::makeDoubleSlider(layout, "roll", 0.0, -M_PI, M_PI, 0.01);
        pitch = QtHelper::makeDoubleSlider(layout, "pitch", 0.0, -M_PI, M_PI, 0.01);
        yaw = QtHelper::makeDoubleSlider(layout, "yaw", 0.0, -M_PI, M_PI, 0.01);

        w = QtHelper::makeSlider(layout, "w", 640, 10, 2000);
        h = QtHelper::makeSlider(layout, "h", 480, 10, 2000);
        focal_length = QtHelper::makeDoubleSlider(layout, "focal length", 55.0, 1.0, 100.0, 0.1);

        connect(x, SIGNAL(valueChanged(double)), worker, SLOT(updatePose()));
        connect(y, SIGNAL(valueChanged(double)), worker, SLOT(updatePose()));
        connect(z, SIGNAL(valueChanged(double)), worker, SLOT(updatePose()));
        connect(roll, SIGNAL(valueChanged(double)), worker, SLOT(updatePose()));
        connect(pitch, SIGNAL(valueChanged(double)), worker, SLOT(updatePose()));
        connect(yaw, SIGNAL(valueChanged(double)), worker, SLOT(updatePose()));
        connect(w, SIGNAL(valueChanged(int)), worker, SLOT(updatePose()));
        connect(h, SIGNAL(valueChanged(int)), worker, SLOT(updatePose()));
        connect(focal_length, SIGNAL(valueChanged(double)), worker, SLOT(updatePose()));

        makeThread();
        worker->moveToThread(private_thread_);
        connect(private_thread_, SIGNAL(finished()), private_thread_, SLOT(deleteLater()));

        private_thread_->start();
    }
}

void VirtualCamera::toggle(bool on)
{
    if(on && !worker->timer_->isActive()) {
        worker->timer_->start();
    } else if(!on && worker->timer_->isActive()) {
        worker->timer_->stop();
    }
}

void VirtualCamera::messageArrived(ConnectorIn *source)
{
    // NO INPUTS
}

void VirtualCamera::import(const QString& filename)
{
    if(!filename.isNull()) {
        if(worker->import(filename)) {
            QtHelper::clearLayout(additional_layout_);
            worker->provider_->insert(additional_layout_);

            file_dialog_->setText(filename);

        } else {
            file_dialog_->setText("Import");
        }
    } else {
        file_dialog_->setText("Import");
    }
}

void VirtualCamera::importDialog()
{
    QString filename = QFileDialog::getOpenFileName(0, "Input", worker->state.last_path_, "All files (*.*)");

    import(filename);
}

Memento::Ptr VirtualCamera::getState() const
{
    boost::shared_ptr<VirtualCameraWorker::State> memento(new VirtualCameraWorker::State((VirtualCamera*) this));
    *memento = worker->state;

    return memento;
}

void VirtualCamera::setState(Memento::Ptr memento)
{
    boost::shared_ptr<VirtualCameraWorker::State> m = boost::dynamic_pointer_cast<VirtualCameraWorker::State> (memento);
    assert(m.get());

    worker->state = *m;

    x->setDoubleValue(m->pos[0]);
    y->setDoubleValue(m->pos[1]);
    z->setDoubleValue(m->pos[2]);

    w->setValue(m->w);
    h->setValue(m->h);
    focal_length->setDoubleValue(m->focal_length);

    worker->provider_->setState(m->sub_state);

}
