#include "ctrl_factory.h"
#include <gui/terra_trainer_window.h>
#include <gui/terra_sub_window.h>
/// UI
#include <ui_terra_toolbar.h>
#include <ui_terra_trainer_window.h>
#include <ui_terra_classes_window.h>
#include <ui_terra_preferences.h>


/// CONTROLLERS
#include <controllers/ctrl_cmpcore_bridge.h>
#include <controllers/ctrl_main_menu.h>
#include <controllers/ctrl_map_view.h>
#include <controllers/ctrl_toolpanel.h>
#include <controllers/ctrl_class_edit.h>
#include <controllers/ctrl_preferences.h>

using namespace Controller;

void CtrlFactory::produdeBridgeController(TerraTrainerWindow *mainWindow)
{
    if(mainWindow->core_.get() == NULL) {
        std::cerr << "Compuatation Core not yet initialized, therefore cancelling Controller Bridge initialization!";
        return;
    }

    CMPCoreBridge *ctrl = new CMPCoreBridge(mainWindow->core_);

    IDPtr entry(Bridge, Ptr(ctrl));
    mainWindow->controllers_.insert(entry);
}

void CtrlFactory::produceMapViewController(TerraTrainerWindow *mainWindow)
{
    Ui::TerraTrainerWindow *ui = mainWindow->ui_;
    CMPCoreBridge::Ptr  br = Controller::to<CMPCoreBridge>(mainWindow->controllers_[Bridge]);

    if(br == NULL) {
        std::cerr << "Bridge Controller not yet initialzed, cancelling Map View Controller init!" << std::endl;
        return;
    }

    CtrlMapView *ctrl = new CtrlMapView(ui->mapView, br);
    QObject::connect(br.get(), SIGNAL(imageLoaded()),         ctrl, SLOT(imageUpdate()));
    QObject::connect(br.get(), SIGNAL(trainingFinished()),    ctrl, SLOT(trainingFinished()));
    QObject::connect(br.get(), SIGNAL(feedbackFinished()),    ctrl, SLOT(feedbackFinished()));
    QObject::connect(br.get(), SIGNAL(classRemoved(int)),     ctrl, SLOT(classRemoved(int)));
    QObject::connect(br.get(), SIGNAL(classUpdate(int,int)),  ctrl, SLOT(classUpdated(int,int)));
    QObject::connect(br.get(), SIGNAL(colorUpdate(int)),      ctrl, SLOT(colorUpdate(int)));

    IDPtr entry(MapView, Ptr(ctrl));
    mainWindow->controllers_.insert(entry);
}

void CtrlFactory::produceMenuController(TerraTrainerWindow *mainWindow)
{
    Ui::TerraTrainerWindow *ui = mainWindow->ui_;
    CtrlMenu               *ctrl = new CtrlMenu(ui->menu);
    CMPCoreBridge::Ptr  br = Controller::to<CMPCoreBridge>(mainWindow->controllers_[Bridge]);
    CtrlMapView::Ptr        mv = Controller::to<CtrlMapView>(mainWindow->controllers_[MapView]);

    if(mv == NULL){
        std::cerr << "Map View Controller not yet initialized, therefore cancelling Menu Controller initialization!" << std::endl;
        return;
    }

    QAction::connect(ui->action_LoadImage,          SIGNAL(triggered()), ctrl, SLOT(loadImage()));
    QAction::connect(ui->action_LoadClassifier,     SIGNAL(triggered()), ctrl, SLOT(loadClassifier()));
    QAction::connect(ui->action_SaveClassifier,     SIGNAL(triggered()), ctrl, SLOT(saveClassifier()));
    QAction::connect(ui->action_SaveClassifier_raw, SIGNAL(triggered()), ctrl, SLOT(saveClassifierRaw()));
    QAction::connect(ui->action_ZoomIn,             SIGNAL(triggered()), ctrl, SLOT(zoomIn()));
    QAction::connect(ui->action_ZoomOut,            SIGNAL(triggered()), ctrl, SLOT(zoomOut()));
    QAction::connect(ui->action_ZoomReset,          SIGNAL(triggered()), ctrl, SLOT(zoomReset()));
    QObject::connect(mv.get(),                      SIGNAL(zoomUpdated(double)), ctrl, SLOT(zoomUpdate(double)));
    QObject::connect(ctrl,                          SIGNAL(zoom(double)), mv.get(), SLOT(zoom(double)));
    QObject::connect(ctrl,                          SIGNAL(loadImage(QString)),      br.get(), SLOT(loadImage(QString)));
    QObject::connect(ctrl,                          SIGNAL(loadClassifier(QString)), br.get(), SLOT(loadClassifier(QString)));
    QObject::connect(ctrl,                          SIGNAL(saveClassifier(QString)), br.get(), SLOT(saveClassifier(QString)));
    QObject::connect(ctrl,                          SIGNAL(saveClassifierRaw(QString)), br.get(), SLOT(saveClassifierRaw(QString)));

    IDPtr entry(Menu, Ptr(ctrl));
    mainWindow->controllers_.insert(entry);
}

void CtrlFactory::produceToolBarController(TerraTrainerWindow *mainWindow)
{
    Ui::ToolPanel          *tp = mainWindow->tool_panel_ui_;
    CMPCoreBridge::Ptr  br = Controller::to<CMPCoreBridge>(mainWindow->controllers_[Bridge]);
    CtrlMapView::Ptr        mv = Controller::to<CtrlMapView>(mainWindow->controllers_[MapView]);

    if(br == NULL) {
        std::cerr << "Bridge Controller not yet initialized, therefore cancelling ToolPanel Controller initialization!" << std::endl;
        return;
    }

    if(mv == NULL) {
        std::cerr << "Map View Controller not yet initialized, therefore cancelling ToolPanel Controller initialization!" << std::endl;
        return;
    }

    CtrlToolPanel *ctrl = new CtrlToolPanel(mainWindow->tool_panel_window_, br);
    ctrl->setupUI(tp);

    /// BUTTON CONNECTIONS
    QPushButton::connect(tp->zoomIn,    SIGNAL(clicked()),           ctrl,           SLOT(zoomIn()));
    QPushButton::connect(tp->zoomOut,   SIGNAL(clicked()),           ctrl,           SLOT(zoomOut()));

    QPushButton::connect(tp->addBoxes,  SIGNAL(clicked(bool)),       ctrl,           SLOT(buttonAdd(bool)));
    QPushButton::connect(tp->movBoxes,  SIGNAL(clicked(bool)),       ctrl,           SLOT(buttonMov(bool)));
    QPushButton::connect(tp->delBoxes,  SIGNAL(clicked(bool)),       ctrl,           SLOT(buttonDel(bool)));
    QPushButton::connect(tp->selBoxes,  SIGNAL(clicked(bool)),       ctrl,           SLOT(buttonSel(bool)));
    QPushButton::connect(ctrl,          SIGNAL(uncheckAdd(bool)),    tp->addBoxes,   SLOT(setChecked(bool)));
    QPushButton::connect(ctrl,          SIGNAL(uncheckMov(bool)),    tp->movBoxes,   SLOT(setChecked(bool)));
    QPushButton::connect(ctrl,          SIGNAL(uncheckDel(bool)),    tp->delBoxes,   SLOT(setChecked(bool)));
    QPushButton::connect(ctrl,          SIGNAL(uncheckSel(bool)),    tp->selBoxes,   SLOT(setChecked(bool)));
    QComboBox::connect(tp->classes,     SIGNAL(currentIndexChanged(int)), ctrl,      SLOT(classChanged(int)));
    QComboBox::connect(tp->features,    SIGNAL(currentIndexChanged(int)), ctrl,      SLOT(featuChanged(int)));

    QObject::connect(br.get(),          SIGNAL(classifierReloaded()),ctrl,           SLOT(classifierLoaded()));
    QObject::connect(br.get(),          SIGNAL(classAdded(int)),     ctrl,           SLOT(classAdded(int)));
    QObject::connect(br.get(),          SIGNAL(classRemoved(int)),   ctrl,           SLOT(classRemoved(int)));
    QObject::connect(br.get(),          SIGNAL(classUpdate(int,int)),ctrl,           SLOT(classUpdated(int,int)));
    QObject::connect(br.get(),          SIGNAL(colorUpdate(int)),    ctrl,           SLOT(colorUpdate(int)));
    QObject::connect(tp->addBoxes,      SIGNAL(clicked()),           mv.get(),       SLOT(activateAdd()));
    QObject::connect(tp->movBoxes,      SIGNAL(clicked()),           mv.get(),       SLOT(activateMove()));
    QObject::connect(tp->delBoxes,      SIGNAL(clicked()),           mv.get(),       SLOT(activateDelete()));
    QObject::connect(tp->selBoxes,      SIGNAL(clicked()),           mv.get(),       SLOT(activateSelect()));
    QObject::connect(tp->trash,         SIGNAL(clicked()),           mv.get(),       SLOT(activateTrash()));
    QObject::connect(tp->selAll,        SIGNAL(clicked()),           mv.get(),       SLOT(selectAll()));
    QObject::connect(tp->deselAll,      SIGNAL(clicked()),           mv.get(),       SLOT(deselectAll()));
    QObject::connect(tp->compile,       SIGNAL(clicked()),           ctrl,           SLOT(buttonCompute()));
    QObject::connect(mv.get(),          SIGNAL(zoomUpdated(double)), ctrl,           SLOT(zoomUpdate(double)));
    QObject::connect(ctrl,              SIGNAL(zoom(double)),        mv.get(),       SLOT(zoom(double)));
    QObject::connect(ctrl,              SIGNAL(classSelected(int)),  mv.get(),       SLOT(changeClass(int)));
    QObject::connect(ctrl,              SIGNAL(compute()),           mv.get(),       SLOT(compute()));
    QObject::connect(br.get(),          SIGNAL(computationFinished()), ctrl,         SLOT(computationFinished()));
    QDoubleSpinBox::connect(tp->zoomBox,SIGNAL(valueChanged(double)),mv.get(),       SLOT(zoom(double)));
    QDoubleSpinBox::connect(mv.get(),   SIGNAL(zoomUpdated(double)), tp->zoomBox,    SLOT(setValue(double)));
    QDoubleSpinBox::connect(tp->sizeBox,SIGNAL(valueChanged(double)),mv.get(),       SLOT(size(double)));
    QDoubleSpinBox::connect(mv.get(),   SIGNAL(sizeUpdated(double)), tp->sizeBox,    SLOT(setValue(double)));

    IDPtr entry(ToolPanel, Ptr(ctrl));
    mainWindow->controllers_.insert(entry);
}

void CtrlFactory::produceClassEdController(TerraTrainerWindow *mainWindow)
{
    Ui::TerraClasses    *tc = mainWindow->classes_ui_;
    CMPCoreBridge::Ptr   br = Controller::to<CMPCoreBridge>(mainWindow->controllers_[Bridge]);

    if(br == NULL) {
        std::cerr << "Bridge Controller not yet initialized, therefore cancelling ToolPanel Controller initialization!" << std::endl;
        return;
    }

    CtrlClassEdit           *ctrl = new CtrlClassEdit(mainWindow->classes_window_, br);
    ctrl->setupUI(mainWindow->classes_ui_);

    QLineEdit::connect(tc->className,  SIGNAL(textEdited(QString)), ctrl,       SLOT(nameEdited(QString)));
    QLineEdit::connect(tc->classID,    SIGNAL(textEdited(QString)), ctrl,       SLOT(IDEdited(QString)));

    QComboBox::connect(tc->classColor, SIGNAL(currentIndexChanged(int)), ctrl, SLOT(colorIndex(int)));

    QPushButton::connect(tc->addClass, SIGNAL(clicked()),    ctrl,              SLOT(accept()));
    QPushButton::connect(tc->delClass, SIGNAL(clicked()),    ctrl,              SLOT(remove()));
    QPushButton::connect(ctrl,         SIGNAL(enableDel(bool)), tc->delClass,   SLOT(setEnabled(bool)));
    QPushButton::connect(ctrl,         SIGNAL(enableAdd(bool)), tc->addClass,   SLOT(setEnabled(bool)));

    QTableWidget::connect(tc->tableClasses, SIGNAL(cellClicked(int,int)), ctrl, SLOT(cellClicked(int,int)));
    QObject::connect(br.get(), SIGNAL(classifierReloaded()), ctrl, SLOT(classesLoaded()));

    IDPtr entry(Class, Ptr(ctrl));
    mainWindow->controllers_.insert(entry);
}

void CtrlFactory::produceSettingController(TerraTrainerWindow *mainWindow)
{
    Ui::TerraPreferences    *tf = mainWindow->preferences_ui_;
    CMPCoreBridge::Ptr   br = Controller::to<CMPCoreBridge>(mainWindow->controllers_[Bridge]);
    CtrlToolPanel::Ptr   tp = Controller::to<CtrlToolPanel>(mainWindow->controllers_[ToolPanel]);

    if(br == NULL) {
        std::cerr << "Bridge Controller not yet initialized, therefore cancelling Settings Controller initialization!" << std::endl;
        return;
    }

    if(tp  == NULL) {
        std::cerr << "Toolpanel Controller not yet initialized, cancelling Settings Controller initialization!" << std::endl;
        return;
    }

    CtrlPreferences         *ctrl = new CtrlPreferences(mainWindow->preferences_window_, br);
    ctrl->setupUI(tf);

    /// ORB
    QCheckBox::connect(tf->orbBox, SIGNAL(clicked(bool)), ctrl, SLOT(orbOppChanged(bool)));
    QSpinBox::connect (tf->spinBox_levelOrb, SIGNAL(valueChanged(int)),      ctrl, SLOT(orbLevelChanged(int)));
    QSpinBox::connect (tf->spinBox_scaleOrb, SIGNAL(valueChanged(double)),   ctrl, SLOT(orbScaleChanged(double)));
    QSpinBox::connect (tf->spinBox_WTAOrb,   SIGNAL(valueChanged(int)),      ctrl, SLOT(orbWTA_KChanged(int)));
    QSpinBox::connect (tf->spinBox_patchOrb, SIGNAL(valueChanged(int)),      ctrl, SLOT(orbPatchChanged(int)));

    /// SURF
    QCheckBox::connect(tf->surfBox,               SIGNAL(clicked(bool)),     ctrl, SLOT(surfOppChanged(bool)));
    QSpinBox::connect (tf->spinBox_octavesSurf,   SIGNAL(valueChanged(int)), ctrl, SLOT(surfOctavesChanged(int)));
    QSpinBox::connect (tf->spinBox_layersSurf,    SIGNAL(valueChanged(int)), ctrl, SLOT(surfOctaveLayersChanged(int)));
    QCheckBox::connect(tf->checkBox_extendedSurf, SIGNAL(clicked(bool)),     ctrl, SLOT(surfExtendeChanged(bool)));

    /// SIFT
    QCheckBox::connect(tf->siftBox,               SIGNAL(clicked(bool)),        ctrl, SLOT(siftOppChanged(bool)));
    QSpinBox::connect (tf->spinBox_magSift,       SIGNAL(valueChanged(double)), ctrl, SLOT(siftMagnificationChanged(double)));
    QSpinBox::connect (tf->spinBox_octavesSift,   SIGNAL(valueChanged(int)),    ctrl, SLOT(siftOctavesChanged(int)));
    QCheckBox::connect(tf->checkBox_angSift,      SIGNAL(clicked(bool)),        ctrl, SLOT(siftRecalcAnglesChanged(bool)));
    QCheckBox::connect(tf->checkBox_normSift,     SIGNAL(clicked(bool)),        ctrl, SLOT(siftNormalizeChanged(bool)));

    /// BRISK
    QCheckBox::connect(tf->briskBox,          SIGNAL(clicked(bool)),             ctrl, SLOT(briskOppChanged(bool)));
    QComboBox::connect(tf->combo_numberBrisk, SIGNAL(editTextChanged(QString)),  ctrl, SLOT(briskNumberListChanged(QString)));
    QComboBox::connect(tf->combo_radiusBrisk, SIGNAL(editTextChanged(QString)),  ctrl, SLOT(briskRadiusListChanged(QString)));
    QSpinBox::connect( tf->spinBox_dMaxBrisk, SIGNAL(valueChanged(double)),      ctrl, SLOT(briskdMaxChanged(double)));
    QSpinBox::connect( tf->spinBox_dMinBrisk, SIGNAL(valueChanged(double)),      ctrl, SLOT(briskdMinChanged(double)));
    tf->combo_numberBrisk->installEventFilter(ctrl);
    tf->combo_radiusBrisk->installEventFilter(ctrl);

    /// BRIEF
    QCheckBox::connect(tf->briefBox,       SIGNAL(clicked(bool)),                 ctrl, SLOT(briefOppChanged(bool)));
    QComboBox::connect(tf->comboBox_Brief, SIGNAL(currentIndexChanged(QString)),  ctrl, SLOT(briefBytesChanged(QString)));

    /// FREAK
    QCheckBox::connect(tf->freakBox,                SIGNAL(clicked(bool)),        ctrl, SLOT(freakOppChanged(bool)));
    QSpinBox::connect( tf->spinBox_octavesFreak,    SIGNAL(valueChanged(int)),    ctrl, SLOT(freakOctavesChanged(int)));
    QSpinBox::connect( tf->spinBox_patternFreak,    SIGNAL(valueChanged(double)), ctrl, SLOT(freakPatternScaleChanged(double)));
    QCheckBox::connect(tf->checkBox_oriNormFreak,   SIGNAL(clicked(bool)),        ctrl, SLOT(freakOriNormChanged(bool)));
    QCheckBox::connect(tf->checkBox_scaleNormFreak, SIGNAL(clicked(bool)),        ctrl, SLOT(freakScaleNormChanged(bool)));

    /// PRESET
    QObject::connect(tp.get(), SIGNAL(featuSelected(QString)), ctrl, SLOT(activateSetting(QString)));
    tp->sync();
    /// TREE


    IDPtr entry(Preferences, Ptr(ctrl));
    mainWindow->controllers_.insert(entry);
}
