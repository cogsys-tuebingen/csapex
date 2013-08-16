#include "ctrl_preferences.h"

#include <ui_terra_preferences.h>
#include <QKeyEvent>

CtrlPreferences::CtrlPreferences(QMainWindow *preferences, CMPCoreBridge::Ptr bridge) :
    dirty_(CLEAN),
    bridge_(bridge)
{
}

void CtrlPreferences::setupUI(Ui::TerraPreferences *ui)
{
    brisk_number_list_ = ui->combo_numberBrisk;
    brisk_radius_list_ = ui->combo_radiusBrisk;

    /// SYNC GUI WITH PARAMS
    /// ORB
    ui->orbBox->setChecked(orb_.opp);
    ui->spinBox_levelOrb->setValue(orb_.levels);
    ui->spinBox_scaleOrb->setValue(orb_.scale);
    ui->spinBox_patchOrb->setValue(orb_.patchSize);
    ui->spinBox_WTAOrb->setValue(orb_.WTA_K);
    /// SURF
    ui->surfBox->setChecked(surf_.opp);
    ui->spinBox_layersSurf->setValue(surf_.octaveLayers);
    ui->spinBox_octavesSurf->setValue(surf_.octaves);
    ui->checkBox_extendedSurf->setChecked(surf_.extended);
    /// SIFT
    ui->siftBox->setChecked(sift_.opp);
    ui->spinBox_magSift->setValue(sift_.magnification);
    ui->spinBox_octavesSift->setValue(sift_.octaves);
    ui->checkBox_angSift->setChecked(sift_.recalculateAngles);
    /// BRISK
    ui->briskBox->setChecked(brisk_.opp);
    ui->spinBox_dMaxBrisk->setValue(brisk_.dMax);
    ui->spinBox_dMinBrisk->setValue(brisk_.dMin);
    /// BRIEF
    ui->briefBox->setChecked(brief_.opp);
    /// FREAK
    ui->freakBox->setChecked(freak_.opp);
    ui->checkBox_oriNormFreak->setChecked(freak_.orientationNormalized);
    ui->checkBox_scaleNormFreak->setChecked(freak_.scaleNormalized);
    ui->spinBox_patternFreak->setValue(freak_.patternScale);
    ui->spinBox_octavesFreak->setValue(freak_.octaves);
    /// TREE
    ui->spinBox_treeMaxDepth->setValue(forest_.max_depth);
    ui->spinBox_treeMinSampels->setValue(forest_.min_samples);
    ui->spinBox_treeRegression->setValue(forest_.regression);
    ui->checkBox_treeSurrogates->setChecked(forest_.surrogates);
    ui->spinBox_treeCategories->setValue(forest_.max_categories);
    ui->checkBox_treeVariableImportance->setChecked(forest_.variable_importance);
    ui->spinBox_treeNactiveVariables->setValue(forest_.nactive_variables);
    ui->spinBox_treeMaxTrees->setValue(forest_.max_trees);
    ui->spinBox_treeAccuracy->setValue(forest_.accurracy);
    /// KEYPOINT
    ui->spinBox_sizeKeypoint->setValue(key_.size);
    ui->spinBox_angleKeypoint->setValue(key_.angle);
    ui->checkBox_softCrop->setChecked(key_.soft_crop);
    /// FEEDBACK




    bridge_->setKeyPointParams(key_);
    bridge_->setForestParams(forest_);
    bridge_->setGridParams(grid_);
    bridge_->setQuadParams(quad_);

}

void CtrlPreferences::orbOppChanged(bool checked)
{
    orb_.opp = checked;
}

void CtrlPreferences::orbLevelChanged(int levels)
{
    orb_.levels = levels;
}

void CtrlPreferences::orbScaleChanged(double scale)
{
    orb_.scale = scale;
}

void CtrlPreferences::orbWTA_KChanged(int wta_k)
{
    orb_.WTA_K = wta_k;
}

void CtrlPreferences::orbPatchChanged(int size)
{
    orb_.patchSize = size;
}

void CtrlPreferences::surfOppChanged(bool checked)
{
    surf_.opp = checked;
}

void CtrlPreferences::surfOctavesChanged(int octaves)
{
    surf_.octaves = octaves;
}

void CtrlPreferences::surfOctaveLayersChanged(int layers)
{
    surf_.octaveLayers = layers;
}

void CtrlPreferences::surfExtendeChanged(bool checked)
{
    surf_.extended = checked;
}

void CtrlPreferences::siftOppChanged(bool checked)
{
    sift_.opp = checked;
}

void CtrlPreferences::siftMagnificationChanged(double magnification)
{
    sift_.magnification = magnification;
}

void CtrlPreferences::siftOctavesChanged(int ocataves)
{
    sift_.octaves = ocataves;
}

void CtrlPreferences::siftNormalizeChanged(bool checked)
{
    sift_.normalize = checked;
}

void CtrlPreferences::siftRecalcAnglesChanged(bool checked)
{
    sift_.recalculateAngles = checked;
}

void CtrlPreferences::briskOppChanged(bool checked)
{
    brisk_.opp = checked;
}

void CtrlPreferences::briskRadiusListChanged(QString value)
{
    QRegExp rx("[^0-9\\.]");
    value.replace(rx, "");
    brisk_radius_list_->setEditText(value);
}

void CtrlPreferences::briskNumberListChanged(QString value)
{
    QRegExp rx("[^0-9\\.]");
    value.replace(rx, "");
    brisk_number_list_->setEditText(value);
}

void CtrlPreferences::briskdMaxChanged(double dMax)
{
    brisk_.dMax = dMax;
}

void CtrlPreferences::briskdMinChanged(double dMIn)
{
    brisk_.dMin = dMIn;
}

void CtrlPreferences::briefOppChanged(bool checked)
{
    brief_.opp = checked;
}

void CtrlPreferences::briefBytesChanged(QString bytes)
{
    brief_.bytes = bytes.toInt();
}

void CtrlPreferences::freakOppChanged(bool checked)
{
    freak_.opp = checked;
}

void CtrlPreferences::freakPatternScaleChanged(double scale)
{
    freak_.patternScale = scale;
}

void CtrlPreferences::freakScaleNormChanged(bool checked)
{
    freak_.scaleNormalized = checked;
}

void CtrlPreferences::freakOriNormChanged(bool checked)
{
    freak_.orientationNormalized = checked;
}

void CtrlPreferences::freakOctavesChanged(int octaves)
{
    freak_.octaves = octaves;
}

void CtrlPreferences::keypointSizeChanged(double size)
{
    key_.size = size;
 }

void CtrlPreferences::keypointAngleChanged(double angle)
{
    key_.angle = angle;
}

void CtrlPreferences::keypointCropChanged(bool crop)
{
    key_.soft_crop = crop;
}

void CtrlPreferences::forest_depthChanged(int depth)
{
    forest_.max_depth = depth;
}

void CtrlPreferences::forest_samplesChanged(int samples)
{
    forest_.min_samples = samples;
}

void CtrlPreferences::forest_regressionChanged(double value)
{
    forest_.regression = value;
}

void CtrlPreferences::forest_surrogatesChanged(bool checked)
{
    forest_.surrogates = checked;
}

void CtrlPreferences::forest_categoriesChanged(int amount)
{
    forest_.max_categories = amount;
}

void CtrlPreferences::forest_importanceChanged(bool checked)
{
    forest_.variable_importance = checked;
}

void CtrlPreferences::forest_nactivesChanged(int amount)
{
    forest_.nactive_variables = amount;
}

void CtrlPreferences::forest_maxTreesChanged(int trees)
{
    forest_.max_trees = trees;
}

void CtrlPreferences::forest_accuracyChanged(double accuracy)
{
    forest_.accurracy = accuracy;
}

void CtrlPreferences::feedback_gridCellChanged(int size)
{
    grid_.cell_height = size;
    grid_.cell_width  = size;
    dirty_ = GRID_DIRTY;
}

void CtrlPreferences::feedback_gridHeightChanged(int height)
{
    grid_.height = height;
    dirty_ = GRID_DIRTY;
}

void CtrlPreferences::feedback_gridWidthChanged(int width)
{
    grid_.width = width;
    dirty_ = GRID_DIRTY;
}

void CtrlPreferences::feedback_quadMinSizeChanged(int size)
{
    quad_.min_height = size;
    quad_.min_width  = size;
    dirty_ = QUAD_DIRTY;
}

void CtrlPreferences::feedback_quadMinProbChanged(double prob)
{
    quad_.min_prob = prob;
    dirty_ = QUAD_DIRTY;
}


void CtrlPreferences::applyExtratorParams(QString setting)
{
    if(setting == "ORB")
        bridge_->setExtractorParams(orb_);
    if(setting == "SURF")
        bridge_->setExtractorParams(surf_);
    if(setting == "SIFT")
        bridge_->setExtractorParams(sift_);
    if(setting == "BRIEF")
        bridge_->setExtractorParams(brief_);
    if(setting == "BRISK")
        bridge_->setExtractorParams(brisk_);
    if(setting == "FREAK")
        bridge_->setExtractorParams(freak_);
  ///  if(setting == "LTP")

    bridge_->setKeyPointParams(key_);
    bridge_->setForestParams(forest_);
    dirty_ = DIRTY;
}

void CtrlPreferences::applyGridParams()
{
    if(dirty_ == DIRTY) {
        bridge_->setGridParams(grid_);
        dirty_ = QUAD_DIRTY;
    } else if(dirty_ == GRID_DIRTY) {
        bridge_->setGridParams(grid_);
        dirty_ = CLEAN;
    }
}

void CtrlPreferences::applyQuadParams()
{
    if(dirty_ == DIRTY) {
        bridge_->setQuadParams(quad_);
        dirty_ = GRID_DIRTY;
    } else if(dirty_ == QUAD_DIRTY) {
        bridge_->setQuadParams(quad_);
        dirty_ = CLEAN;
    }
}

bool CtrlPreferences::eventFilter(QObject *obj, QEvent *event)
{
    QComboBox *combo = dynamic_cast<QComboBox*>(obj);
    if(combo != NULL) {
        if( event->type() == QEvent::KeyPress) {
            int index = combo->currentIndex();
            if(index > -1) {
                QKeyEvent *key = static_cast<QKeyEvent*>(event);
                if(key->key() == Qt::Key_Delete)
                    combo->removeItem(index);
                if(key->key() == Qt::Key_Enter) {
                    updateBriskNumbers();
                    updateBriskRadi();
                }
            }
        }
    }

    if(brisk_number_list_->count() != brisk_.numberList.size())
        updateBriskNumbers();
    if(brisk_radius_list_->count() != brisk_.radiusList.size())
        updateBriskRadi();

    return QObject::eventFilter(obj, event);
}

void CtrlPreferences::updateBriskNumbers()
{
    brisk_.numberList.clear();
    for(int i = 0 ; i < brisk_number_list_->count() ; i++)
        brisk_.numberList.push_back(brisk_number_list_->itemText(i).toInt());
}

void CtrlPreferences::updateBriskRadi()
{
    brisk_.radiusList.clear();
    for(int i = 0 ; i < brisk_radius_list_->count() ; i++)
        brisk_.radiusList.push_back(brisk_radius_list_->itemText(i).toFloat());
}
