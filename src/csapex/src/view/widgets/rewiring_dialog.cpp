/// HEADER
#include <csapex/view/widgets/rewiring_dialog.h>

/// COMPONENT
#include <csapex/view/designer/graph_view.h>

/// SYSTEM
#include <QLabel>
#include <QKeyEvent>
#include <QListView>
#include <QVBoxLayout>

using namespace csapex;


RewiringDialog::RewiringDialog(QWidget *parent, Qt::WindowFlags f)
    : QDialog(parent, f)
{
    makeUI();
}

void RewiringDialog::makeUI()
{
    setWindowIcon(QIcon(":/pencil.png"));
    setWindowTitle("Change Node");

    setFocusPolicy(Qt::StrongFocus);
    setModal(true);

    QVBoxLayout* layout = new QVBoxLayout;
    setLayout(layout);

    //GraphView* view_old = new GraphView();
}


void RewiringDialog::finish()
{
    if(true) {
        Q_EMIT reject();
    } else {
        Q_EMIT accept();
    }
}

/// MOC
#include "../../../include/csapex/view/widgets/moc_rewiring_dialog.cpp"
