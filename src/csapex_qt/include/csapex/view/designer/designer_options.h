#ifndef DESIGNER_OPTIONS_H
#define DESIGNER_OPTIONS_H

/// COMPONENT
#include <csapex_qt_export.h>

/// SYSTEM
#include <QObject>

namespace csapex
{
class Designer;
class Settings;
class GraphView;

class CSAPEX_QT_EXPORT DesignerOptions : public QObject
{
    Q_OBJECT

public:
    DesignerOptions(Settings& settings, Designer* designer);

    void setup(GraphView* view);

public:
    bool isGridEnabled() const;
    bool isGridLockEnabled() const;
    bool isSchematicsEnabled() const;
    bool isGraphComponentsEnabled() const;
    bool isThreadsEnabled() const;
    bool isFrequencyEnabled() const;
    bool isMinimapEnabled() const;
    bool isDebug() const;

    bool areSignalConnectionsVisible() const;
    bool areMessageConnectionsVisibile() const;
    bool areActiveConnectionsVisible() const;
    bool areInactiveConnectionsVisibile() const;

public Q_SLOTS:
    void enableGrid(bool);
    void enableGridLock(bool enabled);
    void enableSchematics(bool);
    void displayGraphComponents(bool);
    void displayThreads(bool);
    void displayFrequency(bool);
    void displayMinimap(bool);
    void enableDebug(bool);

    void displaySignalConnections(bool);
    void displayMessageConnections(bool);
    void displayActiveConnections(bool);
    void displayInactiveConnections(bool);

Q_SIGNALS:
    void gridEnabled(bool);
    void gridLockEnabled(bool);
    void minimapEnabled(bool);
    void debugEnabled(bool);
    void schematicsEnabled(bool);
    void graphComponentsEnabled(bool);
    void threadsEnabled(bool);
    void frequencyEnabled(bool);

    void signalsEnabled(bool);
    void messagesEnabled(bool);
    void activeEnabled(bool);
    void inactiveEnabled(bool);

private:
    void displayConnections(const std::string& key, bool display);

private:
    Settings& settings_;
    Designer* designer_;
};

}  // namespace csapex

#endif  // DESIGNER_OPTIONS_H
