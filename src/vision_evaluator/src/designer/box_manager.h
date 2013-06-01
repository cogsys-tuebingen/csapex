#ifndef BOX_MANAGER_H
#define BOX_MANAGER_H

class BoxManager
{
public:
    static BoxManager& instance() {
        static BoxManager inst;
        return inst;
    }

protected:
    BoxManager();
    BoxManager(const BoxManager& copy);
    BoxManager& operator = (const BoxManager& assign);
};

#endif // BOX_MANAGER_H
