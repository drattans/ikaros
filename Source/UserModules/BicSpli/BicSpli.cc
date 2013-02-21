#include "BicSpli.h"
#include "interp_h.h"
#include "math.h"

using namespace ikaros;

void
BicSpli::Init()
{
  tapo = GetInputArray("TAPO");//Position of target (x,y)
  pupo = GetInputArray("PUPO");//Position of pushable center (x,y)
  pin = GetInputArray("PIN");//Proprioseptic input (a1, a2, a3, a4)
  npo = GetOutputArray("NPO");//New finger position (x,y)
  el = GetFloatValue("EL");//Elevation of finger
  pi = atan(1.f)*4.f;//Pi
}

void
BicSpli::Tick()
{
  bool pmode = false;//Push mode
  bool pdone = false;//Push done
  float fb [2];//Feedback
  float fipo [2];//Position of finger (x,y)
  float pupoo [2];//Old position of pushable center (x,y)
  float radious = 25.f;//Good if it could find this by itself, but can be put to >21 for the time being
  float pl = 15.f;//Push length
  float te = 5.f;//Tolerated error
  float na = (float)rand()/((float)RAND_MAX/(2*pi))-pi;//New angle, initiated as a random float from -pi to pi
  float tpa;//Target-pushable angle
  float tpd = sqrt(pow((pupo[0]-tapo[0]), 2) + pow((pupo[1]-tapo[1]),2));//Target-pushable distance
  if(tpd == 0){//If tapo == pupo -> pushable at target //tpd < ml/2?
    printf("Error (tpd): %f\n", tpd);
  }
  else if((pupo[1]-tapo[1]) > 0){
    tpa = acos(-(pupo[0]-tapo[0])/tpd);
  }
  else{
    tpa = acos((pupo[0]-tapo[0])/tpd)-pi;
  }
  if(tpa<-pi || tpa>pi){
    printf("Error (tpa): %f\n", tpa);
  }
  else if(tpd == 0){
    printf("Error (tpd): %f\n", tpd);
  }
  else{//The interessting part
    ATC(pin);
    fipo[0] = pin[0];
    fipo[1] = pin[2];
    if(pdone==true){
      pdone = false;
      float fbd = sqrt(pow((pupo[0]-tapo[0]), 2) + pow((pupo[1]-tapo[1]),2));
      if(fbd == 0){
	printf("Did not move (tpd): %f\n", fbd);
      }
      else if((pupoo[1]-pupo[1]) > 0){
	fb[0] = acos(-(pupoo[0]-pupo[0])/fbd);
      }
      else{
	fb[0] = acos((pupoo[0]-pupo[0])/fbd)-pi;
      }
      fb[1] = na;//attack angle
      Manage(fb);
      Spline_interp pushEffect(xx, yy);
      na = pushEffect.interp(tpa);
    }
    if(abs(npo[0]-fipo[0])>te || abs(npo[1]-fipo[1])>te){
      pmode = false;
      //pdone = false;//Unnecessary at the moment
      npo[0] = pupo[0] + radious*cos(na);
      npo[1] = pupo[1] + radious*sin(na);
    }
    else{
      pmode = true;
      npo[0] = pupo[0] + (radious-pl)*cos(na);
      npo[1] = pupo[1] + (radious-pl)*sin(na);
      pupoo[0] = pupo[0];
      pupoo[1] = pupo[1];
      pdone = true;
    }
    if(pmode = true){
      el = 0.f;
    }
    else{
      el = 40.f;
    }
  }
}

void
BicSpli::Manage(float fb[])
{
  float te = pi/36.f;//Tolerated error
  float nat;//New angle test
  /***
   * Add new data point?
   ***/
  Spline_interp pushEffectManage(xx, yy);
  nat = pushEffectManage.interp(fb[0]);
  if(abs(nat-fb[1])>te){
    int t=0;
    //if(xx.size()>0){
    for(int i=0; i<xx.size(); ++i){
      if(fb[0]<xx[i]){
	t=i;
	break;
      }
    }
    //}
    xx.insert(xx.begin()+t, fb[0]);
    yy.insert(yy.begin()+t, fb[1]);
  }
  /*
    if(pruning condition){
    int oi;
    xx.erase(oi);
    yy.erase(oi);
    }
  */
}

void
BicSpli::ATC(float *aip)
{//Converts the angles to cartesian coordinates
  float y1 = 126.f; //Arm length (mm)
  float y2 = 112.f; //Forearm length (mm)
  float k = -6.f + el;//Arm base from table (mm)
  float fe = 0.f;//Extention of finger (mm)
  float tna [2];
  float iccp [2];
  tna[0] = (pin[0] - 180.f)*pi/180.f;
  tna[1] = (180.f - pin[2])*pi/180.f;
  iccp[0] = tna[0]+pi/2;
  iccp[1] = sqrt((pow(y1,2) + pow(y2,2) - 2*y1*y2*cos(pi-tna[1])) - pow(k,2)) + fe;
  aip[0] = iccp[1]*cos(iccp[0]);
  aip[2] = iccp[1]*sin(iccp[0]);
}

static InitClass init("BicSpli", &BicSpli::Create, "Source/UserModules/BicSpli/");
