/// HEADER
#include "export_file.h"

/// PROJECT

#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/connection_type.h>
#include <csapex/model/message.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <QFileDialog>

CSAPEX_REGISTER_CLASS(csapex::ExportFile, csapex::Node)

using namespace csapex;

ExportFile::ExportFile()
    : connector_(NULL)
{
    addTag(Tag::get("Output"));
    addTag(Tag::get("General"));
    setIcon(QIcon(":/terminal.png"));

    suffix_ = 0;
}

void ExportFile::fill(QBoxLayout *layout)
{
    if(connector_ == NULL) {
        connector_ = addInput<connection_types::AnyMessage>("Anything");

        setSynchronizedInputs(true);

        file_dialog_ = new QPushButton("Export");
        QObject::connect(file_dialog_, SIGNAL(pressed()), this, SLOT(exportDialog()));

        QVBoxLayout* nested = new QVBoxLayout;
        nested->addWidget(file_dialog_);

        additional_layout_ = new QHBoxLayout;
        nested->addLayout(additional_layout_);

        layout->addLayout(nested);

    }
}

void ExportFile::setExportPath(const QString& path)
{
    if(!path.isNull() && !path.isEmpty()) {
        state.path_ = path.toStdString();
        if(additional_layout_) {
            file_dialog_->setText(path);

        } else {
            file_dialog_->setText("Export");
        }
    } else {
        file_dialog_->setText("Export");
    }
}

void ExportFile::exportDialog()
{
    QString path = QFileDialog::getExistingDirectory(NULL, "Output directory", state.path_.c_str(), QFileDialog::ShowDirsOnly);

    if(!path.isEmpty()) {
        setExportPath(path);
    }
}


void ExportFile::allConnectorsArrived()
{
    if(state.path_.empty()) {
        return;
    }

    ConnectionType::Ptr msg = connector_->getMessage<ConnectionType>();

    QDir dir(state.path_.c_str());
    if(!dir.exists()) {
        QDir().mkdir(state.path_.c_str());
    }

    std::stringstream ss;
    ss << "_" << suffix_;
    msg->writeRaw(state.path_, ss.str());

    ++suffix_;
}

Memento::Ptr ExportFile::getState() const
{
    return boost::shared_ptr<State>(new State(state));
}

void ExportFile::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    assert(m.get());

    state = *m;

    if(!state.path_.empty()) {
        setExportPath(state.path_.c_str());
    }
}


void ExportFile::State::writeYaml(YAML::Emitter& out) const {
    out << YAML::Key << "path" << YAML::Value << path_;
}
void ExportFile::State::readYaml(const YAML::Node& node) {
    if(node.FindValue("path")) {
        node["path"] >> path_;
    }
}
