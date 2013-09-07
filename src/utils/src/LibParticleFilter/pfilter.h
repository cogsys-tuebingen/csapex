// Author: Andreas Masselli

#include <vector>
//#include <string>

#include <rangle.h>

struct Pose {
	float posX;
	float posY;
    float posZ;
    Angle oriZ; // in radians, normalized from 0 to 2*pi
};

struct Particle {
	Pose state;

    double prob;
    double cumuProb; // cumulative probability
};

class ParticleFilter {
public:
    ParticleFilter(float xmin, float xmax,
                   float ymin, float ymax,
                   float zmin, float zmax,
                   Angle oriZleft, Angle oriZright,
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
    // boundaries
    const float mXmin, mXmax, mYmin, mYmax, mZmin, mZmax;
    Angle mOriZleft, mOriZright;
	
    float mDiffuseTrans;
    float mDiffuseOri;
    std::vector<Particle> mParticles;
    std::vector<Particle> newParticles;

    double confidence;

    void initParticles(int noOfParticles = -1);
    void resample();
    void move();
};
