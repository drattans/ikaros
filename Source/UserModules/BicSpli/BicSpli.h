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
  float * tapot;
  float * pupo;
  float * pin;
  float * indir;
  float * ininf;
  float * npo;
  float * npoo;
  float * pupoo;
  float * fb;//Feedback
  float * fbo;//Feedback old
  float * fipo;//Position of finger (x,y)
  float * tg;
  float * el;
  float * nns;
  float * norin;
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
  float rota;
  float shift2;
  float shift3;

  int noR;
  int tlc;
  int tlc2;
  int tlcm;
  int inin;
  int ercou;
  int ercouli;

  bool just;
  bool first;
  bool pmode;//Push mode
  bool pdone;//Push done
  bool ddone;//Finished recently?
  bool initi;//Push initialised
  bool firstM;
  bool protact;

  vector<VecDoub> xx;
  vector<VecDoub> yy;
  //vector<VecDoub> temPo;

  vector<vector<bool>> ok;

 BicSpli(Parameter * p) : Module(p) {}
  virtual ~BicSpli();

  void Init();
  void Tick();
  void NSH();
  void Manage(float fb[]);
  void ATC(float *aip);

  float findTA(float fbd);
  float findTPA(float fbd);
  float mtci(float angtch);

  int PosinX(float a, int b);
};

#endif

