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
    yaml << YAML::Key << "view_pos";
    yaml << YAML::Value << YAML::BeginSeq
         << designer_.ui->scrollArea->horizontalScrollBar()->value()
         << designer_.ui->scrollArea->verticalScrollBar()->value()
         << YAML::EndSeq;
}

void DesignerIO::loadSettings(YAML::Node &doc)
{
    QWidget* window = designer_.window();

    if(exists(doc, "view_pos")) {
        int sx, sy;
        doc["view_pos"][0] >> sx;
        doc["view_pos"][1] >> sy;

        designer_.setView(sx, sy);
    }
}
