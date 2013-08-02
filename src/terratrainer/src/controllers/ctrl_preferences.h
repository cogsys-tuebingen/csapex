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
    void orbLevelChanged         (int levels);
    void orbScaleChanged         (double scale);
    void orbWTA_KChanged         (int wta_k);
    void orbPatchChanged         (int size);
    /// SURF
    void surfOppChanged          (bool checked);
    void surfOctavesChanged      (int octaves);
    void surfOctaveLayersChanged (int layers);
    void surfExtendeChanged      (bool checked);
    /// SIFT
    void siftOppChanged          (bool checked);
    void siftMagnificationChanged(double magnification);
    void siftOctavesChanged      (int ocataves);
    void siftNormalizeChanged    (bool checked);
    void siftRecalcAnglesChanged (bool checked);
    /// BRISK
    void briskOppChanged         (bool checked);
    void briskRadiusListChanged  (QString value);
    void briskNumberListChanged  (QString value);
    void briskdMaxChanged        (double dMax);
    void briskdMinChanged        (double dMIn);
    /// BRIEF
    void briefOppChanged         (bool checked);
    void briefBytesChanged       (QString bytes);
    /// FREAK
    void freakOppChanged         (bool checked);
    void freakPatternScaleChanged(double scale);
    void freakScaleNormChanged   (bool checked);
    void freakOriNormChanged     (bool checked);
    void freakOctavesChanged     (int octaves);

    /// GENERAL
    void activateSetting(QString setting);

protected:
    bool eventFilter(QObject *obj, QEvent *event);

private:
    QComboBox *brisk_radius_list_;
    QComboBox *brisk_number_list_;

    CMPCoreBridge::Ptr  bridge_;
    CMPParamsORB            orb_;
    CMPParamsSURF           surf_;
    CMPParamsSIFT           sift_;
    CMPParamsBRIEF          brief_;
    CMPParamsBRISK          brisk_;
    CMPParamsFREAK          freak_;

    /// BRISK SPECIAL
    void updateBriskNumbers();
    void updateBriskRadi();
};

#endif // CTRL_PREFERENCES_H
