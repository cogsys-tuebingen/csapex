/// HEADER
#include <csapex/core/designerio.h>

/// PROJECT
#include <csapex/view/designer.h>

#include "ui_designer.h"

/// SYSTEM
#include <QMessageBox>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <utils_yaml/yamlplus.h>
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

    yaml << YAML::Key << "view_pos";
    yaml << YAML::Value << YAML::BeginSeq
         << designer_.ui->scrollArea->horizontalScrollBar()->value()
         << designer_.ui->scrollArea->verticalScrollBar()->value()
         << YAML::EndSeq;
}

void DesignerIO::loadSettings(YAML::Node &doc)
{
    QWidget* window = designer_.window();

    int w = window->width();
    int h = window->height();
    int x = window->pos().x();
    int y = window->pos().y();

    if(exists(doc, "window_size")) {
        doc["window_size"][0] >> w;
        doc["window_size"][1] >> h;
    }

    if(exists(doc, "window_pos")) {
        doc["window_pos"][0] >> x;
        doc["window_pos"][1] >> y;
    }
    window->setGeometry(x,y,w,h);

    if(exists(doc, "view_pos")) {
        int sx, sy;
        doc["view_pos"][0] >> sx;
        doc["view_pos"][1] >> sy;

        designer_.setView(sx, sy);
    }
}
