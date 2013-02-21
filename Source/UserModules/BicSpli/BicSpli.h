#ifndef BicSpli_
#define BicSpli_

#include "IKAROS.h"
#include <vector> 
//#include "interp_h.h"

using namespace std;

class BicSpli: public Module
{
 public:
  static Module * Create(Parameter * p) { return new BicSpli(p); }

  float * tapo;
  float * pupo;
  float * pin;
  float * npo;
  float el;
  float pi;

  vector<double> xx;
  vector<double> yy;

 BicSpli(Parameter * p) : Module(p) {}
  virtual ~BicSpli() {}

  void Init();
  void Tick();
  void Manage(float fb[]);
  void ATC(float *aip);
};

#endif

