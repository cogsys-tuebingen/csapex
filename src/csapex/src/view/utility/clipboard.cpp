/// HEADER
#include <csapex/view/utility/clipboard.h>

/// SYSTEM
#include <QClipboard>
#include <QMimeData>
#include <QApplication>
#include <yaml-cpp/yaml.h>

using namespace csapex;

bool ClipBoard::canPaste()
{
    const QMimeData* data = QApplication::clipboard()->mimeData();

    static QString valid_types[] { "text/plain", "text/yaml", "xcsapex/node-list" };
    for(const auto& valid_type : valid_types) {
        if(data->hasFormat(valid_type)) {
            return true;
        }
    }

    return false;
}

void ClipBoard::set(const YAML::Node &serialized)
{
    QMimeData* mime = new QMimeData;
    std::stringstream yaml_txt;
    yaml_txt << serialized;

    auto data = QString::fromStdString(yaml_txt.str()).toUtf8();
    mime->setData("text/plain", data);
    mime->setData("text/yaml", data);
    mime->setData("xcsapex/node-list", data);

    QApplication::clipboard()->setMimeData(mime, QClipboard::Clipboard);
}

std::string ClipBoard::get()
{
    const QMimeData* mime = QApplication::clipboard()->mimeData();
    QString data = mime->data("xcsapex/node-list");
    return data.toStdString();
}
