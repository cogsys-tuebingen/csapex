#ifndef YAML_HPP
#define YAML_HPP
#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <computation/cmp_core.h>
#include <controllers/ctrl_cmpcore_bridge.h>
#include <time.h>

namespace CMPYAML {

inline void writeClassIndex(YAML::Emitter &emitter, std::map<int,int> &classes)
{
    emitter << YAML::BeginSeq;
    for(std::map<int,int>::iterator it = classes.begin() ; it != classes.end() ;it++) {
        emitter << it->first;
        emitter << it->second;
    }
    emitter << YAML::EndSeq;
}

inline void writeColors(YAML::Emitter &emitter, std::vector<QColor> &colors)
{

    emitter << YAML::BeginSeq;
    foreach(QColor c, colors) {
        int r,g,b;
        c.getRgb(&r,&g,&b);
        emitter << r << g << b;
    }
    emitter << YAML::EndSeq;
}

inline bool writeForest(YAML::Emitter &emitter, CMPCore *core)
{
    std::string forest_path = core->forestPath();
    std::ifstream forest_in(forest_path.c_str());

    if(!forest_in.is_open()) {
        std::cerr << "Error opening forest for saving!" << std::endl;
        return false;
    }

    std::stringstream buffer;
    buffer << forest_in.rdbuf();
    emitter << buffer.str();
    forest_in.close();
    return true;
}

inline void writeFile(std::ofstream &out, CMPCore *core, CMPCoreBridge *bridge)
{
    /// PREPARE TIMESTAMP
    time_t rawtime;
    struct tm * timeinfo;
    time (&rawtime);
    timeinfo = localtime (&rawtime);

    /// PREPARE CLASSES
    std::map<int,int> class_index;
    bridge->getClassIndex(class_index);
    std::vector<QColor> colors;
    bridge->getColorPalette(colors);

    /// PREPARE CLASSES
    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "date"      << YAML::Value << asctime(timeinfo);    /// only for info
    emitter << YAML::Key << "classes"   << YAML::Value << YAML::Flow;
    writeClassIndex(emitter,class_index);
    emitter << YAML::Key << "colorSize" << YAML::Value << 3;                    /// only for info
    emitter << YAML::Key << "colors"    << YAML::Value << YAML::Flow;
    writeColors(emitter,colors);
    emitter << YAML::Key << "forest"    << YAML::Value;
    writeForest(emitter, core);
    emitter << YAML::EndMap;
    out << emitter.c_str();
}

inline void readClassIndex(YAML::Node &doc, std::map<int,int> &class_index)
{
    const YAML::Node &data = doc["classes"];
    int pos = 0;
    std::pair<int,int> entry;
    for(YAML::Iterator it = data.begin() ; it != data.end() ; it++, pos++) {
        if(pos % 2 == 0) {
            *it >> entry.first;
        } else {
            *it >> entry.second;
             class_index.insert(entry);
        }
    }
}

inline void readColors(YAML::Node &doc, std::vector<QColor> &colors)
{
    const YAML::Node &data = doc["colors"];
    int color_pos = 0;
    QColor color;
    for(YAML::Iterator it = data.begin() ; it != data.end() ; it++, color_pos++) {
        switch(color_pos % 3) {
        case 0:
        {
            int value;
            *it >> value;
            color.setRed(value);
            break;
        }
        case 1:{
            int value;
            *it >> value;
            color.setGreen(value);
            break;
        }
        case 2:{
            int value;
            *it >> value;
            color.setBlue(value);
            colors.push_back(color);
            break;
        }
        }
    }
}

inline void readForest(YAML::Node &doc, std::ofstream &out)
{
    std::string buffer;
    doc["forest"] >> buffer;
    out << buffer;
}

inline void readFile(std::ifstream &in, CMPCore *core, CMPCoreBridge *bridge)
{
    try {
        YAML::Parser    parser(in);
        YAML::Node      doc;
        std::ofstream   out(core->forestPath().c_str());

        if(!out.is_open()) {
            std::cerr << "Error openining the forest work path file for writing!" << std::endl;
            return;
        }

        ///  READ THE DOCUMENT
        parser.GetNextDocument(doc);
        std::string         date;
        std::map<int,int>   class_index;
        std::vector<QColor> colors;
        doc["date"] >> date;
        readClassIndex(doc, class_index);
        readColors(doc, colors);
        readForest(doc, out);
        out.close();

        /// SET THE CLASS STATE
        bridge->load(class_index, colors, core->forestPath());

    } catch (YAML::Exception e) {
        std::cerr << "Error reading classifier file! " << e.what() << std::endl;
        return;
    }
}
}
#endif // YAML_HPP
