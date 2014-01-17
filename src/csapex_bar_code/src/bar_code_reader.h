#ifndef BAR_CODE_READER_
#define BAR_CODE_READER_

/// COMPONENT
#include <csapex/model/boxed_object.h>

namespace csapex
{

class BarCodeReader : public csapex::BoxedObject
{
    Q_OBJECT

public:
    BarCodeReader();

public:
    virtual void fill(QBoxLayout* layout);

public Q_SLOTS:
    virtual void allConnectorsArrived();

private:
    ConnectorIn* in_img;
    ConnectorOut* out_str;
    ConnectorOut* out_roi;

    std::string data_;
    bool lost;
    int forget;
};

}

#endif // BAR_CODE_READER_
