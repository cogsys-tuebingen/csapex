/// HEADER
#include <csapex/view/designer/tutorial_tree_model.h>

/// PROJECT
#include <csapex/core/settings.h>

/// SYSTEM
#include <boost/filesystem.hpp>
#include <boost/version.hpp>
#include <iostream>
#include <QTextStream>
#include <QTreeWidget>

#if (BOOST_VERSION / 100000) >= 1 && (BOOST_VERSION / 100 % 1000) >= 54
namespace bfs = boost::filesystem;
#else
namespace bfs = boost::filesystem3;
#endif

using namespace csapex;

TutorialTreeModel::TutorialTreeModel(Settings& settings)
    : settings_(settings), tree_(nullptr)
{
}

TutorialTreeModel::~TutorialTreeModel()
{
}

void TutorialTreeModel::fill(QTreeWidget *tree)
{
    tree_ = tree;

    std::string cfg_dir = settings_.defaultConfigPath();
    bfs::path tutorial_dir(cfg_dir + "cfg/tutorials/");
    if(bfs::exists(tutorial_dir)) {
        importDirectory(nullptr, tutorial_dir);
    }
}

template<typename Path>
void TutorialTreeModel::importDirectory(QTreeWidgetItem *parent, const Path& p)
{
    bfs::path path = p;

    bool contains_readme = false;
    bool contains_apex_file = false;
    bfs::path readme;
    bfs::path apex_file;

    std::vector<bfs::path> subpaths;

    bfs::directory_iterator end;
    for(auto it = bfs::directory_iterator(path); it != end; ++it) {
        bfs::path sub_path = *it;
        if(bfs::is_directory(sub_path)) {
            subpaths.push_back(sub_path);
        } else {
            if(sub_path.filename() == "README") {
                contains_readme = true;
                readme = sub_path;

            } else if(sub_path.extension() == ".apex") {
                contains_apex_file = true;
                apex_file = sub_path;
            }
        }
    }

    std::sort(subpaths.begin(), subpaths.end());

    if(contains_readme) {
        QFile rmfile(QString::fromStdString(readme.string()));
        ReadMe rm = parseReadMe(rmfile);

        QString id(QString::fromStdString(path.filename().string()));
        if(id == ".") {
            id = "tutorial_id";
        }

        QTreeWidgetItem* item = new QTreeWidgetItem;
        item->setText(0, rm.title);

        if(contains_apex_file) {
            item->setIcon(0, QIcon(":/play.png"));
        } else {
            item->setIcon(0, QIcon(":/folder.png"));
        }

        if(contains_apex_file) {
            item->setData(0, Qt::UserRole, QString::fromStdString(apex_file.string()));
        }

        if(!rm.description.isEmpty()) {
            QTreeWidgetItem* descr = new QTreeWidgetItem();
            descr->setText(0, rm.description);
            if(contains_apex_file) {
                descr->setData(0, Qt::UserRole, QString::fromStdString(apex_file.string()));
            }
            item->addChild(descr);
        }



        if(parent) {
            parent->addChild(item);
        } else {
            tree_->addTopLevelItem(item);
        }


        for(const bfs::path& sub_path : subpaths) {
            importDirectory(item, sub_path);
        }
    }
}

TutorialTreeModel::ReadMe TutorialTreeModel::parseReadMe(QFile &file)
{
    if(!file.open(QIODevice::ReadOnly)) {
        throw std::runtime_error(std::string("cannot open the file ") + file.fileName().toStdString());
    }

    QTextStream in(&file);

    ReadMe res;

    QString first_line = in.readLine();
    while(first_line.isEmpty()) {
        first_line = in.readLine();
    }
    res.title = first_line;

    while(!in.atEnd()) {
        QString next_line = in.readLine();
        if(!next_line.isEmpty()) {
            res.description += next_line;
        }
    }

    file.close();

    return res;
}


