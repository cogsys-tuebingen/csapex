#ifndef BAG_H
#define BAG_H

/// SYSTEM
#include <DBoW2.h>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

/// FORWARD DECLARATION
class Extractor;
class MatchablePose;
class Matchable;

/// SERIALIZATION

namespace boost
{
namespace archive
{

class polymorphic_iarchive;
class polymorphic_oarchive;

}
}

/**
 * @brief The Bag class represents a bag in the bag of words approach
 */
class Bag
{
public:
    static Bag* create();

    int size() const {
        return poses.size();
    }
    void add(MatchablePose* pose);
    void clear() {
        poses.clear();
    }

    virtual void train() = 0;
    virtual double query(Matchable* current_frame, MatchablePose *&out) = 0;

    MatchablePose* operator [](int index) {
        return poses[index];
    }

protected:
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int version) {
    }

protected:
    std::vector<MatchablePose*> poses;

    Bag();
};

template<class TDescriptor, class F>
class BagImplementation : public Bag
{
    friend class Bag;

private:
    BagImplementation();

    void train();
    double query(Matchable* current_frame, MatchablePose *&out);

    void poses2features(const std::vector<MatchablePose*> &poses, std::vector<std::vector<TDescriptor> > &features);
    void matchable2feature(const Matchable* matchable, std::vector<TDescriptor> &feature);

private:
    DBoW2::TemplatedVocabulary<TDescriptor, F> voc;
    DBoW2::TemplatedDatabase<TDescriptor, F> db;

    std::map<DBoW2::EntryId, MatchablePose*> map;


private:
    BOOST_SERIALIZATION_SPLIT_MEMBER()

    friend class boost::serialization::access;

    template<class Archive>
    void save(Archive& ar, const unsigned int version) const {
        std::string tmp = ".tmp";
        {
            voc.save(tmp);

            std::ifstream ifs(tmp.c_str());
            std::string str((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
            ifs.close();

            ar& str;
        }

        {
            db.save(tmp);

            std::ifstream ifs(tmp.c_str());
            std::string str((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
            ifs.close();

            ar& str;
        }
    }

    template<class Archive>
    void load(Archive& ar, const unsigned int version) {
        // @TODO
    }
};


#endif // BAG_H
