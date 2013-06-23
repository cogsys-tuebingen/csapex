/// HEADER
#include "designerio.h"

/// PROJECT
#include "designer.h"
#include "box.h"
#include "box_manager.h"
#include "ui_designer.h"

/// SYSTEM
#include <QMessageBox>
#include <boost/foreach.hpp>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <QScrollBar>

using namespace vision_evaluator;

const std::string DesignerIO::extension = ".vecfg";
const std::string DesignerIO::default_config = "default" + DesignerIO::extension;
const std::string DesignerIO::config_selector = "Configs (*" + DesignerIO::extension + ")";

DesignerIO::DesignerIO()
{
}

void DesignerIO::save(Designer* designer, const std::string& file)
{
    if(file.empty()) {
        std::cerr << "cannot save, no file specified" << std::endl;
        return;
    }

    YAML::Emitter yaml;

    // settings
    yaml << YAML::BeginMap;

    QSize window_size = designer->window()->size();
    yaml << YAML::Key << "window_size";
    yaml << YAML::Value << YAML::BeginSeq << window_size.width() << window_size.height() << YAML::EndSeq;

    QPoint window_pos = designer->window()->pos();
    yaml << YAML::Key << "window_pos";
    yaml << YAML::Value << YAML::BeginSeq << window_pos.x() << window_pos.y() << YAML::EndSeq;

    yaml << YAML::Key << "uuid_map";
    yaml << YAML::Value << BoxManager::instance().uuids;

    yaml << YAML::EndMap;

    QList<vision_evaluator::Box*> boxes = designer->findChildren<vision_evaluator::Box*> ();
    BOOST_FOREACH(vision_evaluator::Box* box, boxes) {
        yaml << *box;
    }

    std::ofstream ofs(file.c_str());
    ofs << yaml.c_str();

    std::cout << "save: " << yaml.c_str() << std::endl;

    BoxManager::instance().setDirty(false);
}

void DesignerIO::load(Designer* designer, const std::string& file)
{
    if(file.empty()) {
        std::cerr << "cannot load, no file specified" << std::endl;
        return;
    }

    QFile test(file.c_str());
    if(!test.exists()) {
        std::cerr << "cannot load " << file << ", doesn't exist" << std::endl;
        return;
    }

    designer->clear();

    std::map<std::string, Box*> loaded_boxes;

    loadBoxShells(designer, file, loaded_boxes);
    loadConnections(designer, file, loaded_boxes);

    BoxManager::instance().setDirty(false);
}

void DesignerIO::loadSettings(Designer* designer, YAML::Node &doc)
{
    QWidget* window = designer->window();

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

    if(doc.FindValue("uuid_map")) {
        doc["uuid_map"] >> BoxManager::instance().uuids;
    }
}

void DesignerIO::loadBoxShells(Designer* designer, const std::string &file, std::map<std::string, Box*>& loaded_boxes)
{
    std::ifstream ifs(file.c_str());
    YAML::Parser parser(ifs);

    YAML::Node doc;

    if(!parser.GetNextDocument(doc)) {
        std::cerr << "cannot read the config" << std::endl;
        return;
    }

    loadSettings(designer, doc);

    while(parser.GetNextDocument(doc)) {
        std::string type, uuid;
        doc["type"] >> type;
        doc["uuid"] >> uuid;

        int x, y;
        doc["pos"][0] >> x;
        doc["pos"][1] >> y;

        std::cout << "create box: " << type << " @ " << x << ", " << y << std::endl;

        Box* box = BoxManager::instance().makeBox(designer->ui->designer, QPoint(x,y), type, uuid);
        loaded_boxes[uuid] = box;
    }
}

void DesignerIO::loadConnections(Designer* designer, const std::string &file,  std::map<std::string, Box*>& loaded_boxes)
{
    std::ifstream ifs(file.c_str());
    YAML::Parser parser(ifs);

    YAML::Node doc;
    if(!parser.GetNextDocument(doc)) {
        /// SKIP THIS CONFIG PART
        std::cerr << "cannot read the config" << std::endl;
        return;
    }

    while(parser.GetNextDocument(doc)) {
        std::string uuid;
        doc["uuid"] >> uuid;

        Box* box = loaded_boxes[uuid];

        Memento::Ptr s = box->getState();
        s->readYaml(doc);
        box->setState(s);
    }
}
