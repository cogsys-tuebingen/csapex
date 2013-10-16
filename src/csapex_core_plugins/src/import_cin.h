#ifndef IMPORT_CIN_H
#define IMPORT_CIN_H

/// PROJECT
#include <csapex/model/boxed_object.h>

namespace csapex {

class ImportCin : public BoxedObject
{
    Q_OBJECT
public:
    ImportCin();

    virtual void fill(QBoxLayout* layout);
    virtual void tick();

private:
    ConnectorOut* connector_;

    std::stringstream buffer;
};

}

#endif // IMPORT_CIN_H
