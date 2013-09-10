#ifndef RANDOM_GENERATOR_H
#define RANDOM_GENERATOR_H
#include <time.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_real.hpp>
/**
 * @brief   The RandomGenerator class is used to save a initialized boost random
 *          generator.
 */
typedef boost::mt19937                     mt32;
typedef boost::mt19937                     mt64;
typedef boost::uniform_int<>               dist_int;
typedef boost::uniform_real<>              dist_real;

template<class Generator, class Distribution, class T>
class RandomGenerator {
public:
    /**
     * @brief RandomGenerator.
     * @param min - type T minimum bound
     * @param max - type T maximum bound
     */
    RandomGenerator(const T &min, const T &max) :
        distribuation_(min, max),
        min_(min),
        max_(max)
    {
        seed_ = time(0);
        random_gen_.seed(seed_);
    }

    RandomGenerator(const T &min, const T &max, const long &seed) :
        distribuation_(min, max),
        seed_(seed),
        min_(min),
        max_(max)
    {
        random_gen_.seed(seed_);
    }

    virtual ~RandomGenerator()
    {
    }

    /**
     * @brief Generate number of initialized boundary.
     * @return
     */
    T generate()
    {
        return distribuation_(random_gen_);
    }

private:
    Generator       random_gen_;
    Distribution    distribuation_;
    long            seed_;
    T               min_;
    T               max_;
};

typedef RandomGenerator<mt32, dist_int, int>     RandomGeneratorInt;
typedef RandomGenerator<mt64, dist_real, double> RandomGeneratorDouble;
typedef RandomGenerator<mt32, dist_real, float>  RandomGeneratorFloat;


#endif // RANDOM_GENERATOR_H
