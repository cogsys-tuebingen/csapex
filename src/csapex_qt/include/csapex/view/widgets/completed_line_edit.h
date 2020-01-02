#ifndef COMPLETED_LINE_EDIT_H
#define COMPLETED_LINE_EDIT_H

/// COMPONENT
#include <csapex_qt/export.h>

/// SYSTEM
#include <QLineEdit>

class QListView;
class QAbstractItemModel;

namespace csapex
{
class CSAPEX_QT_EXPORT CompletedLineEdit : public QLineEdit
{
    Q_OBJECT

public:
    CompletedLineEdit(QWidget* parent = 0);

    std::string getMIME() const;
    std::string getName() const;

public Q_SLOTS:
    void update();
    void setModel(QAbstractItemModel* completer);
    void completeText(const QModelIndex& index);

protected:
    void keyPressEvent(QKeyEvent* e) override;
    void focusOutEvent(QFocusEvent* e) override;
    void focusInEvent(QFocusEvent* e) override;

private:
    QListView* list_view;

    bool was_hidden;

    std::string mime_;
};
}  // namespace csapex

#endif  // COMPLETED_LINE_EDIT_H
