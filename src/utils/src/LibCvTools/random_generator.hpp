#ifndef RANDOM_GENERATOR_H
#define RANDOM_GENERATOR_H
#include <boost/date_time.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/shared_ptr.hpp>

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
    typedef boost::posix_time::ptime                    Time;
    typedef boost::posix_time::microsec_clock           Clock;
    typedef boost::posix_time::time_duration            Duration;
    typedef boost::shared_ptr<RandomGenerator<Generator, Distribution, T> > Ptr;


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

        Time        time(Clock::local_time());
        Duration    dur(time.time_of_day());
        seed_ = dur.total_milliseconds();
        random_gen_.seed(seed_);
    }

    RandomGenerator() :
        distribuation_(0,1),
        min_(0),
        max_(1)
    {
        Time        time(Clock::local_time());
        Duration    dur(time.time_of_day());
        seed_ = dur.total_milliseconds();
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

    T gen(const T)
    {
        return generate();
    }

    void shuffle(std::vector<T> &to_shuffle)
    {
        typename std::vector<T>::iterator first  = to_shuffle.begin();
        typename std::vector<T>::iterator last   = to_shuffle.end();
        typename std::vector<T>::difference_type i,n;
        n = (last-first);
        for (i=n-1; i>0; --i) {
            std::swap (first[i],first[gen(i+1)]);
        }

//        std::random_shuffle(to_shuffle.begin(), to_shuffle.end(), *this);
    }

    void reset(const T &min, const T &max, const long &seed = time(0))
    {
        min_ = min;
        max_ = max;
        distribuation_ = Distribution(min, max);
        seed_  = seed;
        random_gen_.random_gen_.seed(seed_);
    }

    static T generate_on_the_fly(const T &min, const T &max)
    {
        RandomGenerator<Generator, Distribution, T> rand(min, max);
        return rand.generate();
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
