/// HEADER
#include <csapex/core/designerio.h>

/// PROJECT
#include <csapex/view/designer.h>
#include <csapex/model/box.h>
#include "ui_designer.h"

/// SYSTEM
#include <QMessageBox>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <QScrollBar>
#include <sys/types.h>
#include <pwd.h>

using namespace csapex;

DesignerIO::DesignerIO(Designer &designer)
    : designer_(designer)
{
}

void DesignerIO::saveSettings(YAML::Emitter& yaml)
{
    QSize window_size = designer_.window()->size();
    QPoint window_pos = designer_.window()->pos();

    yaml << YAML::Key << "window_size";
    yaml << YAML::Value << YAML::BeginSeq << window_size.width() << window_size.height() << YAML::EndSeq;

    yaml << YAML::Key << "window_pos";
    yaml << YAML::Value << YAML::BeginSeq << window_pos.x() << window_pos.y() << YAML::EndSeq;
}

void DesignerIO::loadSettings(YAML::Node &doc)
{
    QWidget* window = designer_.window();

    int w = window->width();
    int h = window->height();
    int x = window->pos().x();
    int y = window->pos().y();

    if(doc.FindValue("window_size")) {
        doc["window_size"][0] >> w;
        doc["window_size"][1] >> h;
    }

    if(doc.FindValue("window_pos")) {
        doc["window_pos"][0] >> x;
        doc["window_pos"][1] >> y;
    }
    window->setGeometry(x,y,w,h);
}
