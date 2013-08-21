#ifndef CTRL_PREFERENCES_H
#define CTRL_PREFERENCES_H
/// COMPONENT
#include "ctrl_cmpcore_bridge.h"
/// SYSTEM
#include <QObject>
/// DECLARATIONS
class QMainWindow;
class QComboBox;
namespace Ui {
class TerraPreferences;
}


class CtrlPreferences : public QObject
{

    Q_OBJECT

public:
    typedef boost::shared_ptr<CtrlPreferences> Ptr;

    CtrlPreferences(QMainWindow *preferences, CMPCoreBridge::Ptr bridge);

    void setupUI(Ui::TerraPreferences *ui);

public Q_SLOTS:
    /// ORB
    void orbOppChanged           (bool checked);
    void orbColorExtChanged      (bool checked);
    void orbLevelChanged         (int   levels);
    void orbScaleChanged         (double scale);
    void orbWTA_KChanged         (int    wta_k);
    void orbPatchChanged         (int     size);
    void orbCombineChanged       (bool  enable);

    /// SURF
    void surfOppChanged          (bool checked);
    void surfColorExtChanged     (bool checked);
    void surfOctavesChanged      (int  octaves);
    void surfOctaveLayersChanged (int   layers);
    void surfExtendeChanged      (bool checked);
    void surfCombineChanged      (bool  enable);

    /// SIFT
    void siftOppChanged          (bool checked);
    void siftColorExtChanged     (bool checked);
    void siftMagnificationChanged(double magnification);
    void siftOctavesChanged      (int ocataves);
    void siftNormalizeChanged    (bool checked);
    void siftRecalcAnglesChanged (bool checked);
    void siftCombineChanged      (bool  enable);

    /// BRISK
    void briskOppChanged         (bool checked);
    void briskColorExtChanged    (bool checked);
    void briskOctavesChanged     (int  octaves);
    void briskThresholdChanged   (int   thresh);
    void briskScaleChanged       (double scale);
    void briskCombineChanged     (bool  enable);
    /// BRIEF
    void briefOppChanged         (bool  checked);
    void briefColorExtChanged    (bool  checked);
    void briefBytesChanged       (QString bytes);
    /// FREAK
    void freakOppChanged         (bool checked);
    void freakColorExtChanged    (bool checked);
    void freakPatternScaleChanged(double scale);
    void freakScaleNormChanged   (bool checked);
    void freakOriNormChanged     (bool checked);
    void freakOctavesChanged     (int  octaves);
    void freakCombineChanged     (bool  enable);

    /// LBP
    void lbpColorExtChanged     (bool checked);
    /// LTP
    void ltpColorExtChanged     (bool checked);
    void ltpKChanged            (double value);
    void ltpCombineChanged      (bool checked);

    /// KEYPOINT
    void keypointSizeChanged     (double size);
    void keypointAngleChanged    (double angle);
    void keypointCropChanged     (bool crop);
    void keypointOctavesChanged  (int value);
    void keypointCalcAngleChanged(bool enabled);
    /// FOREST
    void forest_depthChanged     (int depth);
    void forest_samplesChanged   (int samples);
    void forest_regressionChanged(double value);
    void forest_surrogatesChanged(bool checked);
    void forest_categoriesChanged(int amount);
    void forest_importanceChanged(bool checked);
    void forest_nactivesChanged  (int amount);
    void forest_maxTreesChanged  (int trees);
    void forest_accuracyChanged  (double accuracy);
    /// FEEDBACK
    void feedback_gridCellChanged(int size);
    void feedback_gridHeightChanged(int height);
    void feedback_gridWidthChanged(int width);
    void feedback_quadMinSizeChanged(int size);
    void feedback_quadMinProbChanged(double prob);

    /// GENERAL
    void applyExtratorParams(QString setting);
    void applyGridParams();
    void applyQuadParams();

Q_SIGNALS:
    void paramsGridApplied();
    void paramsQuadApplied();
    void paramsExtrApplied();

private:
    enum Dirty {CLEAN, DIRTY, GRID_DIRTY, QUAD_DIRTY};
    Dirty dirty_;

    CMPCoreBridge::Ptr      bridge_;
    CMPParamsORB            orb_;
    CMPParamsSURF           surf_;
    CMPParamsSIFT           sift_;
    CMPParamsBRIEF          brief_;
    CMPParamsBRISK          brisk_;
    CMPParamsFREAK          freak_;
    CMPParamsLTP            ltp_;
    CMPParamsLBP            lbp_;
    CMPParamsTSURF          tsurf;
    CMPKeypointParams       key_;
    CMPForestParams         forest_;
    CMPQuadParams           quad_;
    CMPGridParams           grid_;
};

#endif // CTRL_PREFERENCES_H
