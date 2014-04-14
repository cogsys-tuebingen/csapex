#ifndef TEXT_DISPLAY_H_
#define TEXT_DISPLAY_H_

/// PROJECT
#include <csapex/model/boxed_object.h>

/// SYSTEM
#include <QLabel>

namespace csapex {

class TextDisplay : public BoxedObject
{
    Q_OBJECT

public:
    TextDisplay();

    virtual void process();
    virtual void fill(QBoxLayout* layout);

    virtual QIcon getIcon() const;

private:
    ConnectorIn* connector_;

    QLabel* txt_;
};

}

#endif // TEXT_DISPLAY_H_
