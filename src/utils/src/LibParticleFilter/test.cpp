#include "pfilter.h"

#include <iostream>

using namespace std;

class MyParticleFilter: public ParticleFilter
{
public:
    MyParticleFilter(float xmin, float xmax, float ymin, float ymax,
                   unsigned int numParticles, float diffuseTrans = 1.0f)
        :
          ParticleFilter(xmin, xmax, ymin, ymax,
                         numParticles, diffuseTrans)
    {
    }

    virtual double probfunc(Particle* particle) {
        return -particle->state.posX+100.0;
    }
};

int main(int argc, char** argv) {

    const unsigned int numParticles = 60;//atoi(argv[1]);
    const float diffuseTrans = 5.0f;

    cout << "Parameters:\n"
         << "numParticles: " << numParticles << "\n"
         << "PARAM_DIFFUSETRANS: " << diffuseTrans << endl;

    float xmin = -100.0f;
    float xmax =  100.0f;
    float ymin = -100.0f;
    float ymax =  100.0f;

    MyParticleFilter pfilter(xmin, xmax, ymin, ymax, numParticles, diffuseTrans);

    for (int i = 0; i < 100; ++i) {
        pfilter.update();

        // output
//*//        cout.precision(5);
        for (int j=0; j < pfilter.getParticles().size(); ++j) {
            cout << pfilter.getParticles()[j].state.posX << "\t";
            cout << pfilter.getParticles()[j].state.posY << "\n";
        }
        cout << endl;
//*/
        // debug output
        cout << "." << flush;
    }

    return 0;
}

#ifdef RANDOM_TEST
// normal_distribution
#include <iostream>
#include <random>

int main()
{
  const int nrolls=10000;  // number of experiments
  const int nstars=100;    // maximum number of stars to distribute

  std::default_random_engine generator;
  std::normal_distribution<double> distribution(5.0,2.0);

  int p[10]={};

  for (int i=0; i<nrolls; ++i) {
      double number = ParticleFilter::randGaussian()*2.0 + 5.0;//distribution(generator);
//    double number = distribution(generator);
    if ((number>=0.0)&&(number<10.0)) ++p[int(number)];
  }

  std::cout << "normal_distribution (5.0,2.0):" << std::endl;

  for (int i=0; i<10; ++i) {
    std::cout << i << "-" << (i+1) << ": ";
    std::cout << std::string(p[i]*nstars/nrolls,'*') << std::endl;
  }

  return 0;
}
#endif
