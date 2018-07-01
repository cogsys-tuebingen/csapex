#ifndef NOTE_IMPL_HPP
#define NOTE_IMPL_HPP

/// PROJECT
#include <csapex/io/note.h>
#include <csapex/utility/type.h>

namespace csapex
{
template <typename I>
class NoteImplementation : public io::Note
{
protected:
    CLONABLE_IMPLEMENTATION(I);

protected:
    NoteImplementation() : Note(AUUID::NONE)
    {
    }
    NoteImplementation(const AUUID& auuid) : Note(auuid)
    {
    }

    std::string getType() const override
    {
        return typeName();
    }

public:
    static std::string typeName()
    {
        return type2nameWithoutNamespace(typeid(I));
    }
};

}  // namespace csapex

#endif  // NOTE_IMPL_HPP
