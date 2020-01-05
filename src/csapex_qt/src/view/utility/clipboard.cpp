/// HEADER
#include <csapex/view/utility/clipboard.h>

/// PROJECT
#include <csapex/utility/yaml.h>

/// SYSTEM
#include <QClipboard>
#include <QMimeData>
#include <QApplication>

using namespace csapex;

namespace
{
static const QString valid_types[]{ "xcsapex/node-list", "text/yaml", "text/plain" };
}

bool ClipBoard::canPaste()
{
    const QMimeData* data = QApplication::clipboard()->mimeData();

    for (const auto& valid_type : valid_types) {
        if (data->hasFormat(valid_type)) {
            return true;
        }
    }

    return false;
}

void ClipBoard::set(const YAML::Node& serialized)
{
    QMimeData* mime = new QMimeData;
    std::stringstream yaml_txt;
    yaml_txt << serialized;

    auto data = QString::fromStdString(yaml_txt.str()).toUtf8();
    for (const QString& type : valid_types) {
        mime->setData(type, data);
    }

    QApplication::clipboard()->setMimeData(mime, QClipboard::Clipboard);
}

std::string ClipBoard::get()
{
    const QMimeData* mime = QApplication::clipboard()->mimeData();

    for (const QString& type : valid_types) {
        if (mime->hasFormat(type)) {
            QString s = mime->data(type);
            return s.toStdString();
        }
    }

    return {};
}
