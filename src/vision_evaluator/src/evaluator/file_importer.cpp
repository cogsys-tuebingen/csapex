/// HEADER
#include "file_importer.h"

/// COMPONENT
#include "registration.hpp"
#include "messages_default.hpp"

/// PROJECT
#include <designer/box.h>
#include <designer/connector_out.h>
#include <qt_helper.hpp>

/// SYSTEM
#include <boost/foreach.hpp>
#include <QLabel>
#include <QFileDialog>
#include <QTimer>
#include <QtConcurrentRun>
#include <QCheckBox>

STATIC_INIT(FileImporter, generic, {
    SelectorProxy::ProxyConstructor c; \
    c.setType("File Importer"); \
    c.setConstructor(boost::lambda::bind(boost::lambda::new_ptr<SelectorProxyImp<FileImporter> >(), \
    boost::lambda::_1, (QWidget*) NULL)); \
    SelectorProxy::registerProxy(c); \
});

using namespace vision_evaluator;

FileImporterWorker::FileImporterWorker(FileImporter *parent)
    : state(parent), timer_(NULL), output_img_(NULL), output_mask_(NULL)
{
    timer_ = new QTimer();
    timer_->setInterval(100);
    timer_->start();

    state.last_path_ = QDir::currentPath();

    QObject::connect(timer_, SIGNAL(timeout()), this, SLOT(publish()));
}

BoxedObject* FileImporterWorker::getParent()
{
    return state.parent;
}

void FileImporterWorker::State::writeYaml(YAML::Emitter& out) const {
    out << YAML::Key << "path" << YAML::Value << last_path_.toUtf8().constData();

    if(parent->worker->provider_.get() && parent->worker->provider_->getState().get()) {
        out << YAML::Key << "sub_state";
        out << YAML::Value << YAML::BeginMap;
        parent->worker->provider_->getState()->writeYaml(out);
        out << YAML::EndMap;
    }
}
void FileImporterWorker::State::readYaml(const YAML::Node& node) {
    std::string path;
    node["path"] >> path;
    std::cout << "read path: " << path << std::endl;

    last_path_ = QString::fromUtf8(path.c_str());
    parent->import(last_path_);

    if(node.FindValue("sub_state")) {
        const YAML::Node& sub_state_node = node["sub_state"];
        sub_state = parent->worker->provider_->getState();
        sub_state->readYaml(sub_state_node);
//        parent->worker->provider_->setState(sub_state);
    }
}

void FileImporterWorker::publish()
{
    if(provider_.get()) {
        CvMatMessage::Ptr img(new CvMatMessage);
        CvMatMessage::Ptr mask(new CvMatMessage);

        provider_->next(img->value, mask->value);

        output_img_->publish(img);
        output_mask_->publish(mask);
    }
}

bool FileImporterWorker::import(const QString& path)
{
    state.last_path_ = path;
    provider_ = ImageProvider::Ptr(ImageProvider::create(state.last_path_.toUtf8().constData()));

    return provider_.get();
}

void FileImporterWorker::enableBorder(int border)
{
    if(provider_) {
        std::cout << "border: " << border << std::endl;
        provider_->enableBorder(border != 0);
    }
}


FileImporter::FileImporter()
    : worker(NULL)
{
}

FileImporter::~FileImporter()
{
    if(worker != NULL) {
        delete worker;
    }
}

void FileImporter::fill(QBoxLayout* layout)
{
    if(worker == NULL) {
        worker = new FileImporterWorker(this);

        file_dialog_ = new QPushButton("Import");

        QVBoxLayout* nested = new QVBoxLayout;
        nested->addWidget(file_dialog_);

        additional_layout_ = new QHBoxLayout;
        nested->addLayout(additional_layout_);

        layout->addLayout(nested);

        connect(file_dialog_, SIGNAL(pressed()), this, SLOT(importDialog()));

        worker->output_img_ = new ConnectorOut(box_, 0);
        worker->output_img_->setLabel("Image");
        box_->addOutput(worker->output_img_);

        worker->output_mask_ = new ConnectorOut(box_, 1);
        worker->output_mask_->setLabel("Mask");
        box_->addOutput(worker->output_mask_);

        QObject::connect(box_, SIGNAL(toggled(bool)), this, SLOT(toggle(bool)));

        QCheckBox* enable_border = new QCheckBox("enable border (if possible)");
        // @TODO: put border into separate node!!!

        nested->addWidget(enable_border);

        makeThread();
        worker->moveToThread(private_thread_);
        connect(private_thread_, SIGNAL(finished()), private_thread_, SLOT(deleteLater()));
        connect(enable_border, SIGNAL(stateChanged(int)), worker, SLOT(enableBorder(int)));

        private_thread_->start();
    }
}

void FileImporter::toggle(bool on)
{
    if(on && !worker->timer_->isActive()) {
        worker->timer_->start();
    } else if(!on && worker->timer_->isActive()) {
        worker->timer_->stop();
    }
}

void FileImporter::messageArrived(ConnectorIn *source)
{
    // NO INPUT
}

void FileImporter::import(const QString& filename)
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

void FileImporter::importDialog()
{
    QString filename = QFileDialog::getOpenFileName(0, "Input", worker->state.last_path_, "All files (*.*)");

    import(filename);
}

Memento::Ptr FileImporter::getState() const
{
    boost::shared_ptr<FileImporterWorker::State> memento(new FileImporterWorker::State((FileImporter*) this));
    *memento = worker->state;

    return memento;
}

void FileImporter::setState(Memento::Ptr memento)
{
    boost::shared_ptr<FileImporterWorker::State> m = boost::dynamic_pointer_cast<FileImporterWorker::State> (memento);
    assert(m.get());

    worker->state = *m;
    import(worker->state.last_path_);
    worker->provider_->setState(m->sub_state);

}
