/// HEADER
#include "file_importer.h"

/// PROJECT
#include <csapex/box.h>
#include <csapex/connector_in.h>
#include <csapex/connector_out.h>
#include <csapex/qt_helper.hpp>
#include <csapex_core_plugins/string_message.h>
#include <csapex/message_provider_manager.h>

/// SYSTEM
#include <boost/foreach.hpp>
#include <QLabel>
#include <QFileDialog>
#include <QTimer>
#include <QtConcurrentRun>
#include <QCheckBox>
#include <QDirIterator>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::FileImporter, csapex::BoxedObject)

using namespace csapex;
using namespace connection_types;

void FileImporter::State::writeYaml(YAML::Emitter& out) const {
    out << YAML::Key << "path" << YAML::Value << last_path_.toUtf8().constData();

    if(parent->provider_.get() && parent->provider_->getState().get()) {
        out << YAML::Key << "sub_state";
        out << YAML::Value << YAML::BeginMap;
        parent->provider_->getState()->writeYaml(out);
        out << YAML::EndMap;
    }
}
void FileImporter::State::readYaml(const YAML::Node& node) {
    std::string path;
    node["path"] >> path;
    std::cout << "read path: " << path << std::endl;

    last_path_ = QString::fromUtf8(path.c_str());
    parent->import(last_path_);

    if(node.FindValue("sub_state")) {
        const YAML::Node& sub_state_node = node["sub_state"];
        sub_state = parent->provider_->getState();
        sub_state->readYaml(sub_state_node);
        parent->provider_->setState(sub_state);
    }
}

void FileImporter::tick()
{
    if(provider_.get()) {

        Message::Ptr msg = provider_->next();
        if(msg.get()) {
            output_->setType(provider_->getType());
            output_->setLabel(provider_->getType()->name());
            output_->publish(msg);
        }
    }
}

bool FileImporter::doImport(const QString& path)
{
    state.last_path_ = path;
    provider_ = MessageProviderManager::createMessageProvider(path.toUtf8().constData());
    provider_->load(path.toUtf8().constData());

    return provider_.get();
}

void FileImporter::enableBorder(int border)
{
    if(provider_) {
        std::cout << "border: " << border << std::endl;
//        provider_->enableBorder(border != 0);
    }
}


FileImporter::FileImporter()
    : state(this), output_(NULL), additional_layout_(NULL), file_dialog_(NULL)
{
    setIcon(QIcon(":/folder_picture.png"));
}

FileImporter::~FileImporter()
{
}

void FileImporter::fill(QBoxLayout* layout)
{
    if(file_dialog_ == NULL) {
        file_dialog_ = new QPushButton("Import");

        QVBoxLayout* nested = new QVBoxLayout;
        nested->addWidget(file_dialog_);

        additional_layout_ = new QHBoxLayout;
        nested->addLayout(additional_layout_);

        layout->addLayout(nested);

        connect(file_dialog_, SIGNAL(pressed()), this, SLOT(importDialog()));

        optional_input_filename_ = new ConnectorIn(box_, 0);
        optional_input_filename_->setLabel("File (optional)");
        optional_input_filename_->setType(connection_types::StringMessage::make());
        box_->addInput(optional_input_filename_);


        output_ = new ConnectorOut(box_, 0);
        output_->setLabel("Unknown");
        box_->addOutput(output_);

        QObject::connect(box_, SIGNAL(toggled(bool)), this, SLOT(toggle(bool)));

        QCheckBox* enable_border = new QCheckBox("enable border (if possible)");
        // @TODO: put border into separate node!!!

        nested->addWidget(enable_border);

        connect(enable_border, SIGNAL(stateChanged(int)), this, SLOT(enableBorder(int)));
    }
}

void FileImporter::toggle(bool on)
{

}

void FileImporter::messageArrived(ConnectorIn *source)
{
    StringMessage::Ptr msg = boost::dynamic_pointer_cast<StringMessage> (source->getMessage());

    if(msg) {
        std::cout << "got message: " << msg->value << std::endl;
        import(msg->value.c_str());
    }
}

void FileImporter::import(const QString& filename)
{
    if(!filename.isNull() && !filename.isEmpty()) {
        if(additional_layout_ && doImport(filename)) {
            QtHelper::clearLayout(additional_layout_);
            provider_->insert(additional_layout_);

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
    QString filter = QString("Supported files (") + MessageProviderManager::instance().supportedTypes().c_str() + ");;All files (*.*)";
    QString filename = QFileDialog::getOpenFileName(0, "Input", state.last_path_, filter);

    if(!filename.isEmpty()) {
        import(filename);
    }
}

Memento::Ptr FileImporter::getState() const
{
    boost::shared_ptr<FileImporter::State> memento(new FileImporter::State((FileImporter*) this));
    *memento = state;

    return memento;
}

void FileImporter::setState(Memento::Ptr memento)
{
    boost::shared_ptr<FileImporter::State> m = boost::dynamic_pointer_cast<FileImporter::State> (memento);
    assert(m.get());

    state = *m;
    import(state.last_path_);

    if(provider_) {
        provider_->setState(m->sub_state);
    }

}
