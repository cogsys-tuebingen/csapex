#ifndef SELECTABLE_H
#define SELECTABLE_H

/// SYSTEM
#include <boost/signals2.hpp>

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

public:
    boost::signals2::signal<void(bool)> selection;

private:
    bool selected_;
};

}

#endif // SELECTABLE_H
