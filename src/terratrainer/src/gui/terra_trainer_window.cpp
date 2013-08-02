/// COMPONENT
#include "terra_trainer_window.h"
#include "terra_sub_window.h"
#include "ui_terra_trainer_window.h"
#include "ui_terra_toolbar.h"
#include "ui_terra_classes_window.h"
#include "ui_terra_preferences.h"

/// SYSTEM
#include <iostream>
#include <QWidget>
#include <QDir>

TerraTrainerWindow::TerraTrainerWindow(QWidget *parent) :
    QMainWindow(parent),
    ui_(new Ui::TerraTrainerWindow),
    tool_panel_ui_(new Ui::ToolPanel),
    preferences_ui_(new Ui::TerraPreferences),
    classes_ui_(new Ui::TerraClasses),
    tool_panel_window_(new SubWindow(this)),
    preferences_window_(new SubWindow(this)),
    classes_window_(new SubWindow(this))
{
    ui_->setupUi(this);

    tool_panel_ui_->setupUi(tool_panel_window_);
    tool_panel_window_->setVisible(true);

    preferences_ui_->setupUi(preferences_window_);
    preferences_window_->setVisible(false);

    classes_ui_->setupUi(classes_window_);
    classes_window_->setVisible(false);

    init();
    initSignalConnections();

}

TerraTrainerWindow::~TerraTrainerWindow()
{
    delete ui_;
    delete tool_panel_ui_;
    delete preferences_ui_;
    delete classes_ui_;
}

void TerraTrainerWindow::init()
{
    std::cout << "Preparing directories!" << std::endl;
    createDir();
    std::cout << "Starting computation core!" << std::endl;
    core_.reset(new CMPCore());
    QString work_path = QDir::homePath() + "/.terratrainer/tmp";
    core_->setWorkPath(work_path.toUtf8().constData());

    std::cout << "Firing Controllers up!" << std::endl;
    CtrlFactory::produdeBridgeController(this);
    CtrlFactory::produceMapViewController(this);
    CtrlFactory::produceMenuController(this);
    CtrlFactory::produceToolBarController(this);
    CtrlFactory::produceClassEdController(this);
    CtrlFactory::produceSettingController(this);
}

void TerraTrainerWindow::createDir()
{
    QDir dir(QDir::homePath());

    if(!dir.exists(".terratrainer")) {
        std::cout << "Creating .terratrainer in home dir!" << std::endl;
        dir.mkdir(".terratrainer");
        dir.mkdir(".terratrainer/tmp");
    }
}

void TerraTrainerWindow::initSignalConnections()
{
    std::cout << "Initializing intra GUI signals!" << std::endl;
    /// SIGNAL CONNECTIONS NOT NEEDING A CONTROLLER
    QAction::connect(ui_->action_ToolPanel,    SIGNAL(triggered(bool)), tool_panel_window_,    SLOT(setVisible(bool)));
    QAction::connect(tool_panel_window_,       SIGNAL(visible(bool)), ui_->action_ToolPanel,   SLOT(setChecked(bool)));
    QAction::connect(ui_->action_Classes,      SIGNAL(triggered(bool)), classes_window_,       SLOT(setVisible(bool)));
    QAction::connect(classes_window_,          SIGNAL(visible(bool)), ui_->action_Classes,     SLOT(setChecked(bool)));
    QAction::connect(ui_->action_Preferences,  SIGNAL(triggered(bool)), preferences_window_,   SLOT(setVisible(bool)));
    QAction::connect(preferences_window_,      SIGNAL(visible(bool)), ui_->action_Preferences, SLOT(setChecked(bool)));
    QAction::connect(ui_->action_Quit,         SIGNAL(triggered()), this, SLOT(close()));

    /// VISIBILITY AND CHECKING
    ui_->action_ToolPanel->setChecked(tool_panel_window_->isVisible());
    ui_->action_Classes->setChecked(classes_window_->isVisible());
    ui_->action_Preferences->setChecked(preferences_window_->isVisible());
}
