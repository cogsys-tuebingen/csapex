// Author: Andreas Masselli

#include <vector>
#include <string>

struct Pose {
	float posX;
	float posY;
    float oriZ; // in radians
};

struct Particle {
	Pose state;

    double prob;
    double cumuProb; // cumulative probability
};

class ParticleFilter {
public:
    ParticleFilter(float xmin, float xmax, float ymin, float ymax,
                   unsigned int numParticles, float diffuseTrans = 1.0f);

    std::vector<Particle> getParticles() const {
        return mParticles;
    }

    void update();
    static double randGaussian();

    Particle getMean(int noOfParticles = 13) const;

protected:
    virtual double probfunc(Particle* particle) const;

private:
    const float mXmin, mXmax, mYmin, mYmax; // bounding rect
	
    float mDiffuseTrans;
    float mDiffuseOri;
    std::vector<Particle> mParticles;
    std::vector<Particle> newParticles;

    double confidence;

    void initParticles(int noOfParticles = -1);
    void resample();
    void move();
};
