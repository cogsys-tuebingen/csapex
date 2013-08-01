#ifndef TERRA_TRAINER_WINDOW_H
#define TERRA_TRAINER_WINDOW_H

/// COMPONENT
#include <controllers/ctrl_factory.h>
#include <controllers/ctrl_cmpcore_bridge.h>
/// SYSTEM
#include <QMainWindow>
/// DECLARATIONS
class SubWindow;
namespace Ui {
class TerraTrainerWindow;
class ToolPanel;
class TerraPreferences;
class TerraClasses;
}

class TerraTrainerWindow : public QMainWindow
{
    Q_OBJECT

    friend class CtrlFactory;

public:
    explicit TerraTrainerWindow(QWidget *parent = 0);
    ~TerraTrainerWindow();
    
private:
    /// CONTROLLERS
    Controller::Map             ctrls_map_;

    /// UI
    Ui::TerraTrainerWindow      *ui_;
    Ui::ToolPanel               *tool_panel_ui_;
    Ui::TerraPreferences        *preferences_ui_;
    Ui::TerraClasses            *classes_ui_;

    SubWindow                   *tool_panel_window_;
    SubWindow                   *preferences_window_;
    SubWindow                   *classes_window_;

    /// CORE
    CMPCore::Ptr                 core_;
    Controller::Map              controllers_;

    void init();

    void initSignalConnections();
};

#endif // TERRA_TRAINER_WINDOW_H
