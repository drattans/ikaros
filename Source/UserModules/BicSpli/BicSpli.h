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
  float * fbo;//Feedback old
  float * fipo;//Position of finger (x,y)
  float * tg;
  float * el;
  float tpa;//Target-pushable angle
  float tpao;//Target-pushable angle old
  float radious;
  float pl;
  float te;
  float na;
  float h;
  float abcx;
  float abcy;
  float pi;

  int tlc;
  int tlc2;
  int tlcm;

  bool just;
  bool first;
  bool pmode;//Push mode
  bool pdone;//Push done
  bool ddone;//Finished recently?
  bool initi;//Push initialised
  bool firstM;

  VecDoub xx;
  VecDoub yy;
  VecDoub temPo;

  vector<bool> ok;

 BicSpli(Parameter * p) : Module(p) {}
  virtual ~BicSpli();

  void Init();
  void Tick();
  void Manage(float fb[]);
  void ATC(float *aip);

  int PosinX(float a);
};

#endif

