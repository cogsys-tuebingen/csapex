#ifndef SELECTABLE_H
#define SELECTABLE_H

namespace csapex
{

class Selectable {

protected:
    Selectable();

public:
    void setSelected(bool selected);
    bool isSelected() const;

    virtual void selectEvent();
    virtual void deselectEvent();

    bool selected_;
};

}

#endif // SELECTABLE_H
