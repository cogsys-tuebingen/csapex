#include "ctrl_preferences.h"

#include <ui_terra_preferences.h>
#include <QKeyEvent>

CtrlPreferences::CtrlPreferences(CMPCoreBridge::Ptr bridge) :
    dirty_(CLEAN),
    bridge_(bridge)
{
}

void CtrlPreferences::setupUI(Ui::TerraPreferences *ui)
{
    ui_ = ui;

    /// SYNC GUI WITH PARAMS
    /// ORB
    ui_->orbBox->setChecked(orb_.opp);
    ui_->checkBox_colExtOrb->setChecked(orb_.color_extension);
    ui_->spinBox_levelOrb->setValue(orb_.octaves);
    ui_->spinBox_scaleOrb->setValue(orb_.scale);
    ui_->spinBox_patchOrb->setValue(orb_.patch_size);
    ui_->spinBox_WTAOrb->setValue(orb_.WTA_K);
    ui_->checkBox_combineOrb->setChecked(orb_.combine_descriptors);
    ui_->checkBox_maxProbOrb->setChecked(orb_.use_max_prob);

    /// SURF
    ui_->surfBox->setChecked(surf_.opp);
    ui_->checkBox_extendedSurf->setChecked(surf_.color_extension);
    ui_->spinBox_layersSurf->setValue(surf_.octave_layers);
    ui_->spinBox_octavesSurf->setValue(surf_.octaves);
    ui_->checkBox_extendedSurf->setChecked(surf_.extended);
    ui_->checkBox_combineSurf->setChecked(surf_.combine_descriptors);
    ui_->checkBox_maxProbSurf->setChecked(surf_.use_max_prob);

    /// SIFT
    ui_->siftBox->setChecked(sift_.opp);
    ui_->checkBox_angSift->setChecked(sift_.color_extension);
    ui_->spinBox_magSift->setValue(sift_.magnification);
    ui_->spinBox_octavesSift->setValue(sift_.octaves);
    ui_->checkBox_angSift->setChecked(sift_.recalculate_angles);
    ui_->checkBox_maxProbSift->setChecked(sift_.use_max_prob);

    /// BRISK
    ui_->briskBox->setChecked(brisk_.opp);
    ui_->checkBox_colExtBrisk->setChecked(brisk_.color_extension);
    ui_->spinBox_octavesBrisk->setValue(brisk_.octaves);
    ui_->spinBox_threshBrisk->setValue(brisk_.thresh);
    ui_->spinBox_scaleBrisk->setValue(brisk_.scale);
    ui_->checkBox_combineBrisk->setChecked(brisk_.combine_descriptors);
    ui_->checkBox_maxProbBrisk->setChecked(brisk_.use_max_prob);

    /// BRIEF
    ui_->briefBox->setChecked(brief_.opp);
    ui_->checkBox_colExtBrief->setChecked(brief_.color_extension);

    /// FREAK
    ui_->freakBox->setChecked(freak_.opp);
    ui_->checkBox_colExtFreak->setChecked(freak_.color_extension);
    ui_->checkBox_oriNormFreak->setChecked(freak_.orientation_normalized);
    ui_->checkBox_scaleNormFreak->setChecked(freak_.scale_normalized);
    ui_->spinBox_patternFreak->setValue(freak_.pattern_scale);
    ui_->spinBox_octavesFreak->setValue(freak_.octaves);
    ui_->checkBox_combineFreak->setChecked(freak_.combine_descriptors);
    ui_->checkBox_maxProbFreak->setChecked(freak_.use_max_prob);

    /// LBP / LTP
    ui_->checkBox_colExtlbp->setChecked(lbp_.color_extension);
    ui_->checkBox_colExtltp->setChecked(ltp_.color_extension);
    ui_->spinBox_kltp->setValue(ltp_.k);
    ui_->checkBox_combineLTP->setChecked(ltp_.combine_descriptors);

    /// FOREST
    ui_->spinBox_treeMaxDepth->setValue(forest_.max_depth);
    ui_->spinBox_treeMinSampels->setValue(forest_.min_samples);
    ui_->spinBox_treeRegression->setValue(forest_.regression);
    ui_->checkBox_treeSurrogates->setChecked(forest_.surrogates);
    ui_->spinBox_treeCategories->setValue(forest_.max_categories);
    ui_->checkBox_treeVariableImportance->setChecked(forest_.variable_importance);
    ui_->spinBox_treeNactiveVariables->setValue(forest_.nactive_variables);
    ui_->spinBox_treeMaxTrees->setValue(forest_.max_trees);
    ui_->spinBox_treeAccuracy->setValue(forest_.accurracy);

    /// KEYPOINT
    ui_->spinBox_sizeKeypoint->setValue(key_.scale);
    ui_->spinBox_angleKeypoint->setValue(key_.angle);
    ui_->checkBox_softCrop->setChecked(key_.soft_crop);
    ui_->checkBox_calcAngleKeypoint->setChecked(key_.calc_angle);

    /// FEEDBACK
    bridge_->setKeyPointParams(key_);
    bridge_->setForestParams(forest_);
    bridge_->setGridParams(grid_);
    bridge_->setQuadParams(quad_);

}

void CtrlPreferences::write(YAML::Emitter &emitter) const
{
    emitter << YAML::Key << "PREFERENCES" << YAML::Value;
    emitter << YAML::BeginMap;
    orb_.write(emitter);
    surf_.write(emitter);
    sift_.write(emitter);
    brisk_.write(emitter);
    brief_.write(emitter);
    freak_.write(emitter);
    lbp_.write(emitter);
    ltp_.write(emitter);
    key_.write(emitter);
    forest_.write(emitter);
    quad_.write(emitter);
    grid_.write(emitter);
    emitter << YAML::EndMap;
}

void CtrlPreferences::read(const YAML::Node &document)
{
    try {
        const YAML::Node &data = document["PREFERENCES"];
        orb_.read(data);
        surf_.read(data);
        sift_.read(data);
        brisk_.read(data);
        brief_.read(data);
        freak_.read(data);
        lbp_.read(data);
        ltp_.read(data);
        key_.read(data);
        forest_.read(data);
        quad_.read(data);
        grid_.read(data);

    } catch (YAML::Exception e) {
        std::cerr << "Problems reading preferences : '" << e.what() <<"' !" << std::endl;
    }

    setupUI(ui_);
    dirty_ = DIRTY;
}

void CtrlPreferences::readClassfierLoad(const YAML::Node &document)
{
    try {
        const YAML::Node &data = document["CLASSIFIER"];
        if(orb_.read(data))
            bridge_->setExtractorParams(orb_);
        if(surf_.read(data))
            bridge_->setExtractorParams(surf_);
        if(sift_.read(data))
            bridge_->setExtractorParams(sift_);
        if(brisk_.read(data))
            bridge_->setExtractorParams(brisk_);
        if(brief_.read(data))
            bridge_->setExtractorParams(brief_);
        if(freak_.read(data))
            bridge_->setExtractorParams(freak_);
        if(lbp_.read(data))
            bridge_->setExtractorParams(lbp_);
        if(ltp_.read(data))
            bridge_->setExtractorParams(ltp_);

        key_.read(data);
        bridge_->setKeyPointParams(key_);

    } catch (YAML::Exception e) {
        std::cerr << "Problems reading preferences : '" << e.what() <<"' !" << std::endl;
    }

    setupUI(ui_);
    dirty_ = DIRTY;
}

void CtrlPreferences::orbOppChanged(bool checked)
{
    orb_.opp = checked;
}

void CtrlPreferences::orbColorExtChanged(bool checked)
{
    orb_.color_extension = checked;
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
    orb_.patch_size = size;
}

void CtrlPreferences::orbCombineChanged(bool enable)
{
    orb_.combine_descriptors = enable;
}

void CtrlPreferences::orbMaxProbChanged(bool enable)
{
    orb_.use_max_prob = enable;
}

void CtrlPreferences::surfOppChanged(bool checked)
{
    surf_.opp = checked;
}

void CtrlPreferences::surfColorExtChanged(bool checked)
{
    surf_.color_extension = checked;
}

void CtrlPreferences::surfOctavesChanged(int octaves)
{
    surf_.octaves = octaves;
}

void CtrlPreferences::surfOctaveLayersChanged(int layers)
{
    surf_.octave_layers = layers;
}

void CtrlPreferences::surfExtendeChanged(bool checked)
{
    surf_.extended = checked;
}

void CtrlPreferences::surfCombineChanged(bool enable)
{
    surf_.combine_descriptors = enable;
}

void CtrlPreferences::surfMaxProbChanged(bool enable)
{
    surf_.use_max_prob = enable;
}

void CtrlPreferences::siftOppChanged(bool checked)
{
    sift_.opp = checked;
}

void CtrlPreferences::siftColorExtChanged(bool checked)
{
    sift_.color_extension = checked;
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
    sift_.recalculate_angles = checked;
}

void CtrlPreferences::siftCombineChanged(bool enable)
{
    sift_.combine_descriptors = enable;
}

void CtrlPreferences::siftMaxProbChanged(bool enable)
{
    sift_.use_max_prob = enable;
}

void CtrlPreferences::briskOppChanged(bool checked)
{
    brisk_.opp = checked;
}

void CtrlPreferences::briskColorExtChanged(bool checked)
{
    brisk_.color_extension = checked;
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

void CtrlPreferences::briskMaxProbChanged(bool enable)
{
    brisk_.use_max_prob = enable;
}

void CtrlPreferences::briefOppChanged(bool checked)
{
    brief_.opp = checked;
}

void CtrlPreferences::briefColorExtChanged(bool checked)
{
    brief_.color_extension = checked;
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
    freak_.color_extension = checked;
}

void CtrlPreferences::freakPatternScaleChanged(double scale)
{
    freak_.pattern_scale = scale;
}

void CtrlPreferences::freakScaleNormChanged(bool checked)
{
    freak_.scale_normalized = checked;
}

void CtrlPreferences::freakOriNormChanged(bool checked)
{
    freak_.orientation_normalized = checked;
}

void CtrlPreferences::freakOctavesChanged(int octaves)
{
    freak_.octaves = octaves;
}

void CtrlPreferences::freakCombineChanged(bool enable)
{
    freak_.combine_descriptors = enable;
}

void CtrlPreferences::freakMaxProbChanged(bool enable)
{
    freak_.use_max_prob = enable;
}

void CtrlPreferences::keypointSizeChanged(double size)
{
    key_.scale = size;
}

void CtrlPreferences::lbpColorExtChanged(bool checked)
{
    lbp_.color_extension = checked;
}

void CtrlPreferences::ltpColorExtChanged(bool checked)
{
    ltp_.color_extension = checked;
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
