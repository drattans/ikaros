#ifndef BicSpli_
#define BicSpli_

#include "IKAROS.h"
#include <vector> 
//#include "interp_h.h"

using namespace std;

typedef vector<double> VecDoub;

class BicSpli: public Module
{
 public:
  static Module * Create(Parameter * p) { return new BicSpli(p); }

  float * tapo;
  float * pupo;
  float * pin;
  float * npo;
  float * npoo;
  float * pupoo;
  float * fb;//Feedback
  float * fipo;//Position of finger (x,y)
  float * el;
  float radious;
  float pl;
  float te;
  float na;
  float h;
  float abcx;
  float abcy;
  float pi;

  bool pmode;//Push mode
  bool pdone;//Push done
  bool firstM;

  VecDoub xx;
  VecDoub yy;

 BicSpli(Parameter * p) : Module(p) {}
  virtual ~BicSpli();

  void Init();
  void Tick();
  void Manage(float fb[]);
  void ATC(float *aip);
};

#endif

