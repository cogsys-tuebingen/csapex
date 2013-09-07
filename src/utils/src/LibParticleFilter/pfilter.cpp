#include "pfilter.h"

#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdlib>
#include <algorithm>    // std::max

using namespace std;

#define VERBOSE 0
#define BABBLE(stream) {if (VERBOSE) cout << stream << endl;}

#define RANDOM ((double)rand() / (double)(((unsigned int)RAND_MAX)+1) )

bool comp(Particle a, Particle b) {
    return a.prob > b.prob;
}

Particle ParticleFilter::getMean(int noOfParticles) const {
    // get mean pose
    std::vector<Particle> A = this->getParticles();
    std::nth_element(A.begin(), A.begin() + noOfParticles, A.end(), comp);

    Particle mean;
    mean.state.posX = 0.0f;
    mean.state.posY = 0.0f;
    mean.state.posZ = 0.0f;
    mean.prob = 0.0;
    for (int i = 0; i < noOfParticles; ++i) {
        mean.state.posX += A[i].state.posX;
        mean.state.posY += A[i].state.posY;
        mean.state.posZ += A[i].state.posZ;
        mean.prob += A[i].prob;

        //std::cout << A[i].prob <<  " ";
    }
    mean.state.posX /= noOfParticles;
    mean.state.posY /= noOfParticles;
    mean.state.posZ /= noOfParticles;
    mean.prob /= noOfParticles;
    //std::cout << "\n" << std::endl;

    return mean;
}

double ParticleFilter::randGaussian() {
    double r = sqrt( -2.0 * log(1.0-RANDOM) );
    double phi = 6.283185307179586476925286766559 * RANDOM;
    return r * cos(phi);
}

// implemented for debugging
double ParticleFilter::probfunc(Particle* particle) const {
    return particle->state.posX+100.0;
}

ParticleFilter::ParticleFilter(float xmin, float xmax,
                               float ymin, float ymax,
                               float zmin, float zmax,
                               Angle oriZleft, Angle oriZright,
                               unsigned int numParticles, float diffuseTrans)
:
  mXmin(xmin),
  mXmax(xmax),
  mYmin(ymin),
  mYmax(ymax),
  mZmin(zmin),
  mZmax(zmax),
  mOriZleft(oriZleft),
  mOriZrigth(oriZrigth),
  mDiffuseTrans(diffuseTrans),
  mDiffuseOri(1.0f),
  confidence(0.0)
{
	mParticles.reserve(numParticles);
    newParticles.reserve(numParticles);

	for (int i = 0; i < numParticles; ++i) {
		Particle particle;
        mParticles.push_back(particle);
	}

    initParticles();
}

void ParticleFilter::initParticles(int noOfParticles) {
    if (noOfParticles == -1)
        noOfParticles = mParticles.size();

    std::nth_element(mParticles.begin(), mParticles.end() - noOfParticles, mParticles.end(), comp);

    for (int i = mParticles.size() - noOfParticles; i < mParticles.size(); ++i) {
        // x,y-coordinates of particle
        mParticles[i].state.posX = RANDOM * (mXmax-mXmin) + mXmin;
        mParticles[i].state.posY = RANDOM * (mYmax-mYmin) + mYmin;
        mParticles[i].state.posZ = RANDOM * (mZmax-mZmin) + mZmin;
        mParticles[i].state.oriZ = RANDOM * (mOriZleft-mOriZright) + mOriZright;

        // probability of particle
        mParticles[i].prob = 1.0/(double)mParticles.size();
        mParticles[i].cumuProb = (double)(i+1)/(double)mParticles.size();
    }
}

void ParticleFilter::resample() {
    BABBLE("resample");

    for (int i = 0; i < mParticles.size(); ++i) {
        double rn = RANDOM; // random number between 0.0 and 1.0

        BABBLE(rn << ", ");

        // find particle with cumuProb greater than guess
        int k = -1;
        for (int j = 0; j < mParticles.size(); ++j) {
            if (mParticles[j].cumuProb >= rn) {
                k = j;
                j = mParticles.size(); // break
            }
        }

        newParticles[i] = (k > -1 ? mParticles[k] : mParticles[i]);
    }

    // copy
    for (int i = 0; i < mParticles.size(); ++i)
        mParticles[i] = newParticles[i];
}

void ParticleFilter::move() {
    //////////////////////////////////////////////////////
    // drift/diffuse
    BABBLE("drift/diffuse");

/*    double alpha = 0.9;
    if (confidence < 0.0017)
        alpha = 5.0;
*/
    double alpha = 0.3 + 0.7*(1.0-min(1.0, max(0.0, 100.0*(this->confidence-0.002))));

/*
    double alpha = max(0.2, 1.0-2.0*mParticles[mParticles.size()-1].cumuProb);
    alpha = alpha*5.0;
//*/
//    cout << "m: " << this->getMean(13).prob << "\tc: "<< mParticles[mParticles.size()-1].cumuProb << "\ta: " << alpha << endl;
    for (int i = 0; i < mParticles.size(); ++i) {
        // diffuse
        mParticles[i].state.posX += randGaussian() * mDiffuseTrans * alpha;
        mParticles[i].state.posY += randGaussian() * mDiffuseTrans * alpha;
        mParticles[i].state.posZ += randGaussian() * mDiffuseTrans * alpha*0.1;//todo: zalpha
        mParticles[i].state.oriZ += randGaussian() * mDiffuseOri   * alpha;

        // clip to keep within bounding values
        if (mParticles[i].state.posX < mXmin) mParticles[i].state.posX = mXmin;
        if (mParticles[i].state.posX > mXmax) mParticles[i].state.posX = mXmax;

        if (mParticles[i].state.posY < mYmin) mParticles[i].state.posY = mYmin;
        if (mParticles[i].state.posY > mYmax) mParticles[i].state.posY = mYmax;

        if (mParticles[i].state.posZ < mZmin) mParticles[i].state.posZ = mZmin;
        if (mParticles[i].state.posZ > mZmax) mParticles[i].state.posZ = mZmax;
        // todo: handle ori
    }
}

void measure() {
}

void ParticleFilter::update() {

    resample();

    move();

	///////////////////////////////////////////////////////
	// measure
	BABBLE("measure");
    for (int i = 0; i < mParticles.size(); ++i)
        mParticles[i].prob *= probfunc(&mParticles[i]);

    confidence = getMean(3).prob;
//    cout << "confidence: " << confidence << "\t" << endl;
//    cout << "totalProb: " << totalProb << endl;

    if (confidence < 0.002)
        initParticles();

    initParticles(25);

    // apply laplacian smoothing
    for (int i = 0; i < mParticles.size(); ++i)
        mParticles[i].prob += 0.000001;

	////////////////////////////////////////////////////////
    // normalize
	BABBLE("normalize");
    double totalProb = 0.0;
    for (int i = 0; i < mParticles.size(); ++i)
        totalProb += mParticles[i].prob;

    if (totalProb >= 0.0001) {
        double cumuProb = 0.0;
        for (int i = 0; i < mParticles.size(); ++i) {
            mParticles[i].prob /= totalProb; // normalize each prob by the sum of the probs.
            cumuProb += mParticles[i].prob;
            mParticles[i].cumuProb = cumuProb;
		}
	}
	else {
        cerr << "totalProb < 0.0001" << endl;
        initParticles();
    }

//    cout << "cumuProb: " << mParticles[mParticles.size()-1].cumuProb << endl;
}
