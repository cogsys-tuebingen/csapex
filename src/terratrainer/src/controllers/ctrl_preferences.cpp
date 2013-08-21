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
    /// SYNC GUI WITH PARAMS
    /// ORB
    ui->orbBox->setChecked(orb_.opp);
    ui->checkBox_colExtOrb->setChecked(orb_.colorExtension);
    ui->spinBox_levelOrb->setValue(orb_.octaves);
    ui->spinBox_scaleOrb->setValue(orb_.scale);
    ui->spinBox_patchOrb->setValue(orb_.patchSize);
    ui->spinBox_WTAOrb->setValue(orb_.WTA_K);
    ui->checkBox_combineOrb->setChecked(orb_.combine_descriptors);

    /// SURF
    ui->surfBox->setChecked(surf_.opp);
    ui->checkBox_extendedSurf->setChecked(surf_.colorExtension);
    ui->spinBox_layersSurf->setValue(surf_.octaveLayers);
    ui->spinBox_octavesSurf->setValue(surf_.octaves);
    ui->checkBox_extendedSurf->setChecked(surf_.extended);
    ui->checkBox_combineSurf->setChecked(surf_.combine_descriptors);

    /// SIFT
    ui->siftBox->setChecked(sift_.opp);
    ui->checkBox_angSift->setChecked(sift_.colorExtension);
    ui->spinBox_magSift->setValue(sift_.magnification);
    ui->spinBox_octavesSift->setValue(sift_.octaves);
    ui->checkBox_angSift->setChecked(sift_.recalculateAngles);

    /// BRISK
    ui->briskBox->setChecked(brisk_.opp);
    ui->checkBox_colExtBrisk->setChecked(brisk_.colorExtension);
    ui->spinBox_octavesBrisk->setValue(brisk_.octaves);
    ui->spinBox_threshBrisk->setValue(brisk_.thresh);
    ui->spinBox_scaleBrisk->setValue(brisk_.scale);
    ui->checkBox_combineBrisk->setChecked(brisk_.combine_descriptors);

    /// BRIEF
    ui->briefBox->setChecked(brief_.opp);
    ui->checkBox_colExtBrief->setChecked(brief_.colorExtension);

    /// FREAK
    ui->freakBox->setChecked(freak_.opp);
    ui->checkBox_colExtFreak->setChecked(freak_.colorExtension);
    ui->checkBox_oriNormFreak->setChecked(freak_.orientationNormalized);
    ui->checkBox_scaleNormFreak->setChecked(freak_.scaleNormalized);
    ui->spinBox_patternFreak->setValue(freak_.patternScale);
    ui->spinBox_octavesFreak->setValue(freak_.octaves);
    ui->checkBox_combineFreak->setChecked(freak_.combine_descriptors);

    /// LBP / LTP
    ui->checkBox_colExtlbp->setChecked(lbp_.colorExtension);
    ui->checkBox_colExtltp->setChecked(ltp_.colorExtension);
    ui->spinBox_kltp->setValue(ltp_.k);
    ui->checkBox_combineLTP->setChecked(ltp_.combine_descriptors);

    /// FOREST
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
    ui->spinBox_sizeKeypoint->setValue(key_.scale);
    ui->spinBox_angleKeypoint->setValue(key_.angle);
    ui->checkBox_softCrop->setChecked(key_.soft_crop);
    ui->checkBox_calcAngleKeypoint->setChecked(key_.calc_angle);

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

void CtrlPreferences::orbColorExtChanged(bool checked)
{
    orb_.colorExtension = checked;
}

void CtrlPreferences::orbLevelChanged(int levels)
{
    orb_.octaves = levels;
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

void CtrlPreferences::orbCombineChanged(bool enable)
{
    orb_.combine_descriptors = enable;
}

void CtrlPreferences::surfOppChanged(bool checked)
{
    surf_.opp = checked;
}

void CtrlPreferences::surfColorExtChanged(bool checked)
{
    surf_.colorExtension = checked;
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

void CtrlPreferences::surfCombineChanged(bool enable)
{
    surf_.combine_descriptors = enable;
}

void CtrlPreferences::siftOppChanged(bool checked)
{
    sift_.opp = checked;
}

void CtrlPreferences::siftColorExtChanged(bool checked)
{
    sift_.colorExtension = checked;
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

void CtrlPreferences::siftCombineChanged(bool enable)
{
    sift_.combine_descriptors = enable;
}

void CtrlPreferences::briskOppChanged(bool checked)
{
    brisk_.opp = checked;
}

void CtrlPreferences::briskColorExtChanged(bool checked)
{
    brisk_.colorExtension = checked;
}

void CtrlPreferences::briskOctavesChanged(int octaves)
{
    brisk_.octaves = octaves;
}

void CtrlPreferences::briskThresholdChanged(int thresh)
{
    brisk_.thresh = thresh;
}

void CtrlPreferences::briskScaleChanged(double scale)
{
    brisk_.scale = scale;
}

void CtrlPreferences::briskCombineChanged(bool enable)
{
    brisk_.combine_descriptors = enable;
}

void CtrlPreferences::briefOppChanged(bool checked)
{
    brief_.opp = checked;
}

void CtrlPreferences::briefColorExtChanged(bool checked)
{
    brief_.colorExtension = checked;
}

void CtrlPreferences::briefBytesChanged(QString bytes)
{
    brief_.bytes = bytes.toInt();
}

void CtrlPreferences::freakOppChanged(bool checked)
{
    freak_.opp = checked;
}

void CtrlPreferences::freakColorExtChanged(bool checked)
{
    freak_.colorExtension = checked;
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

void CtrlPreferences::freakCombineChanged(bool enable)
{
    freak_.combine_descriptors = enable;
}

void CtrlPreferences::keypointSizeChanged(double size)
{
    key_.scale = size;
}

void CtrlPreferences::lbpColorExtChanged(bool checked)
{
    lbp_.colorExtension = checked;
}

void CtrlPreferences::ltpColorExtChanged(bool checked)
{
    ltp_.colorExtension = checked;
}

void CtrlPreferences::ltpKChanged(double value)
{
    ltp_.k = value;
}

void CtrlPreferences::ltpCombineChanged(bool checked)
{
    ltp_.combine_descriptors = checked;
}

void CtrlPreferences::keypointAngleChanged(double angle)
{
    key_.angle = angle;
}

void CtrlPreferences::keypointCropChanged(bool crop)
{
    key_.soft_crop = crop;
}

void CtrlPreferences::keypointOctavesChanged(int value)
{
    key_.octave = value;
}

void CtrlPreferences::keypointCalcAngleChanged(bool enabled)
{
    key_.calc_angle = enabled;
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
    if(setting == "LBP")
        bridge_->setExtractorParams(lbp_);
    if(setting == "LTP")
        bridge_->setExtractorParams(ltp_);

    bridge_->setKeyPointParams(key_);
    bridge_->setForestParams(forest_);
    dirty_ = DIRTY;

    Q_EMIT paramsExtrApplied();
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

    Q_EMIT paramsGridApplied();
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

    Q_EMIT paramsQuadApplied();
}
