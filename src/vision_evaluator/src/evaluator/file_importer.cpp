/// HEADER
#include "file_importer.h"

/// COMPONENT
#include "registration.hpp"

/// PROJECT
#include <designer/box.h>
#include <designer/connector_out.h>

/// SYSTEM
#include <boost/foreach.hpp>
#include <QLabel>
#include <QFileDialog>
#include <QTimer>

STATIC_INIT(FileImporter, generic, {
                SelectorProxy::ProxyConstructor c;\
                c.setName("File Importer");\
                c.setConstructor(boost::lambda::bind(boost::lambda::new_ptr<SelectorProxyImp<FileImporter> >(), \
                boost::lambda::_1, (QWidget*) NULL)); \
                SelectorProxy::registerProxy(c);\
            });

using namespace vision_evaluator;

FileImporter::FileImporter()
    : output_img_(NULL), output_mask_(NULL), timer_(NULL)
{
}

void FileImporter::fill(QBoxLayout *layout)
{
    if(output_img_ == NULL) {
        file_dialog_ = new QPushButton("Import");

        layout->addWidget(file_dialog_);

        connect(file_dialog_, SIGNAL(pressed()), this, SLOT(importDialog()));

        output_img_ = new ConnectorOut(box_);
        box_->addOutput(output_img_);

        output_mask_ = new ConnectorOut(box_);
        box_->addOutput(output_mask_);

        QObject::connect(box_, SIGNAL(toggled(bool)), this, SLOT(toggle(bool)));

        timer_ = new QTimer();
        timer_->setInterval(100);
        timer_->start();

        makeThread();
        timer_->moveToThread(private_thread_);

        QObject::connect(timer_, SIGNAL(timeout()), this, SLOT(publish()));
    }
}

void FileImporter::toggle(bool on)
{
    if(on && !timer_->isActive()) {
        timer_->start();
    } else if(!on && timer_->isActive()) {
        timer_->stop();
    }
}

void FileImporter::publish()
{
    if(!provider_.isNull()) {
        provider_->next();
    }
}

void FileImporter::importDialog()
{
    QString filename = QFileDialog::getOpenFileName(0, "Input", QDir::currentPath(), "All files (*.*)");

    if(!filename.isNull()) {
        provider_ = QSharedPointer<ImageProvider>(ImageProvider::create(filename.toUtf8().constData()));

        if(!provider_.isNull()) {
            file_dialog_->setText(filename);
            return;
        }
    }

    file_dialog_->setText("Import");
}
