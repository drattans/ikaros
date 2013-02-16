#ifndef BicSpli_
#define BicSpli_

#include "IKAROS.h"
#include <vector> 
#include "interp.h"

using namespace std;

class BicSpli: public Module
{
 public:
  static Module * Create(Parameter * p) { return new BicSpli(p); }

  float * fipo;
  float * pupo;
  float * npo;

  float pi;

  //int n = 360;
  vector<double> xx(0), yy(0);//(n)

 BicSpli(Parameter * p) : Module(p) {}
  virtual ~BicSpli() {}

  void 		Init();
  void 		Tick();
};

#endif

