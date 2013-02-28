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
  el = GetOutputArray("EL");//Elevation of finger
  abcx = GetFloatValue("ABCX", 0.5f);//Arm base coordinate (x)
  abcy = GetFloatValue("ABCY", 1.f);//Arm base coordinate (y)
  pi = atan(1.f)*4.f;//Pi
  h = 525.f;//Camera hight from surface (mm)
  if(!npoo){//=if(npoo == NULL){
    npoo = new float[2];
    npoo[0] = 0.f;//npo of the last tick, x
    npoo[1] = 0.f;//npo of the last tick, y
  }
  pupoo = new float[2];
  pmode = false;//Push mode
  pdone = false;//Push done
  firstM = true;
  fipo = new float[2];
  fb = new float[2];
  radious = 25.f;//Good if it could find this by itself, but can be put to >21 for the time being
  pl = 15.f;//Push length
  te = 10.f;//Tolerated error
  na = (float)rand()/((float)RAND_MAX/(2*pi))-pi;//New angle, initiated as a random float from -pi to pi
}

void
BicSpli::Tick()
{
  //printf("El: %f\n", el);
  tapo[0] = tapo[0] - abcx;
  tapo[1] = abcy - tapo[1];
  pupo[0] = pupo[0] - abcx;
  pupo[1] = abcy - pupo[1];
  float tpa;//Target-pushable angle
  float tpd = sqrt(pow((pupo[0]-tapo[0]), 2) + pow((pupo[1]-tapo[1]),2));//Target-pushable distance
  if(tpd == 0){//If tapo == pupo -> pushable at target //tpd < ml/2?
    printf("Error (tpd): %f\n", tpd);
  }
  else if((pupo[1]-tapo[1]) > 0){
    tpa = - acos(-(pupo[0]-tapo[0])/tpd);
  }
  else{
    tpa = - acos((pupo[0]-tapo[0])/tpd) + pi;
  }
  if(tpa<-pi || tpa>pi){
    printf("Error (tpa): %f\n", tpa);
  }
  else if(tpd == 0){
    printf("Error (tpd): %f\n", tpd);
  }
  else{//The interessting part
    printf("Tpa: %f\n", tpa);
    ATC(pin);
    fipo[0] = pin[0];
    fipo[1] = pin[2];
    //printf("Finger position x: %f, finger position y: %f\n", fipo[0], fipo[1]);
    if(pdone==true){
      printf("\n3\n\n");
      pdone = false;
      float fbd = sqrt(pow((pupo[0]-tapo[0]), 2) + pow((pupo[1]-tapo[1]),2));
      if(fbd == 0){
	printf("Did not move (tpd): %f\n", fbd);
      }
      else if((pupoo[1]-pupo[1]) > 0){
	fb[0] = - acos((pupoo[0]-pupo[0])/fbd);
      }
      else{
	fb[0] = - acos(-(pupoo[0]-pupo[0])/fbd)+pi;
      }
      fb[1] = na;//attack angle
      printf("Effect angle: %F, Attack angle: %f\n", fb[0], fb[1]);
      Manage(fb);
      Spline_interp *pushEffect = new Spline_interp(xx,yy);
      na = pushEffect->interp(tpa);
    }
    if(abs(npoo[0]-fipo[0])>te || abs(npoo[1]-fipo[1])>te){
      pmode = false;
      printf("1\n");
      printf("1: old order x: %f, new position x: %f\n", npoo[0], fipo[0]);
      printf("1: old order y: %f, new position y: %f\n", npoo[1], fipo[1]);
      float ar1 = abs(npoo[0]-fipo[0]);
      float ar2 = abs(npoo[1]-fipo[1]);
      printf("1: Difference x: %f, Difference y: %f, Tolerated error: %f\n", ar1, ar2, te);
      //pdone = false;//Unnecessary at the moment
      float f1 = ((radious*cos(na))/(2.f*h*tan(28.5*pi/180.f)));
      float f2 = ((radious*sin(na))/(2.f*h*tan(21.5*pi/180.f)));
      npo[0] = pupo[0] - f1 + abcx;
      npo[1] = abcy - pupo[1] + f2;
      npoo[0] = (npo[0] - abcx)*(2.f*h*tan(28.5*pi/180.f));
      npoo[1] = (abcy - npo[1])*(2.f*h*tan(21.5*pi/180.f));
      printf("1: old order x: %f, old order y: %f\n", npoo[0], npoo[1]);
      //printf("Pos 1 finger position x: %f, Pos 1 finger position y: %f\n", pupo[0], pupo[1]);
      //printf("Cor 1 finger position x: %f, Cor 1 finger position y: %f\n", f1, f2);
      //printf("Out 1 finger position x: %f, Out 1 finger position y: %f\n", npo[0], npo[1]);
    }
    else{
      pmode = true;
      printf("2\n");
      float f1 = (((radious-pl)*cos(na))/(2*h*tan(28.5*pi/180)));
      float f2 = (((radious-pl)*sin(na))/(2*h*tan(21.5*pi/180)));
      npo[0] = pupo[0] - f1 + abcx;
      npo[1] = abcy - pupo[1] - f2;
      npoo[0] = (npo[0] - abcx)/(2*h*tan(28.5*pi/180));
      npoo[1] = (abcy - npo[1])/(2*h*tan(21.5*pi/180));
      printf("2: old order x: %f, old order y: %f\n", npoo[0], npoo[1]);
      printf("2: new order x: %f, new order y: %f\n", npo[0], npo[1]);
      //npo[0] = (pupo[0] + (radious-pl)*cos(na))/(2*h*tan(28.5*pi/180)) - abcx;
      //npo[1] = abcy - (pupo[1] + (radious-pl)*sin(na))/(2*h*tan(21.5*pi/180));
      //printf("Out 2 finger position x: %f, Out 2 finger position y: %f\n", npo[0], npo[1]);
      pupoo[0] = pupo[0];
      pupoo[1] = pupo[1];
      pdone = true;
    }
    if(pmode == true){
      el[0] = 0.f;
      //printf("3\n");
    }
    else{
      //printf("4\n");
      el[0] = -40.f;
    }
    //printf("El: %f\n", el[0]);
  }
}

void
BicSpli::Manage(float fb[])
{
  printf("Managed!\n");
  float te = pi/18.f;//Tolerated error
  float nat;//New angle test
  /***
   * Add new data point?
   ***/
  if(firstM == true){
    printf("Managed, in loop, %i!\n", xx.size());
    xx.push_back(fb[0]);
    yy.push_back(fb[1]);
    xx.push_back((fb[0]+1.f));//DETTA ÄR JÄTTEFEL!!! MÅSTE ÄNDRAS!!!
    yy.push_back((fb[1]+1.f));//DETTA ÄR JÄTTEFEL!!! MÅSTE ÄNDRAS!!!
    firstM=false;
  }
  printf("Managed, outside loop, %f!\n", xx[(xx.size()-1)]);
  printf("Managed, outside loop, %f!\n", yy[(yy.size()-1)]);
  Spline_interp *pushEffectManage = new Spline_interp(xx,yy);
  printf("Managed 2!\n");
  nat = pushEffectManage->interp(fb[0]);
  printf("Managed 3! Nat: %f\n", nat);
  if(abs(nat-fb[1])>te){
    int t=0;
    //if(xx.size()>0){
    for(int i=0; i<xx.size(); ++i){
      if(fb[0]<xx[i]){
	t=i;
	break;
      }
    }
    printf("Managed 4!\n");
    //}
    xx.insert(xx.begin()+t, fb[0]);
    yy.insert(yy.begin()+t, fb[1]);
  }
  printf("Knowledge: %i\n----------------------------------------------------------------\n", xx.size());
  /*
    if(pruning condition){
    int oi;
    xx.erase(oi);
    yy.erase(oi);
    }
  */
  delete pushEffectManage;
}

void
BicSpli::ATC(float *aip)
{//Converts the angles to cartesian coordinates
  float y1 = 126.f; //Arm length (mm)
  float y2 = 112.f; //Forearm length (mm)
  float k = 5.f + el[0];//Arm base from table (mm)
  float fe = 54.f;//Extention of finger (mm)
  float tna [2];
  float iccp [2];
  tna[0] = (pin[0] - 180.f)*pi/180.f;
  tna[1] = (180.f - pin[2])*pi/180.f;
  iccp[0] = (tna[0]+pi/2);
  iccp[1] = (sqrt((pow(y1,2) + pow(y2,2) - 2*y1*y2*cos(pi-tna[1])) - pow(k,2)) + fe);
  //printf("ATC finger position a: %f, ATC finger position r: %f\n", iccp[0], iccp[1]);
  aip[0] = iccp[1]*cos(iccp[0]);///(2*h*tan(28.5*pi/180)) - abcx;
  aip[2] = iccp[1]*sin(iccp[0]);///(2*h*tan(21.5*pi/180)); abcy - 
}

//void
BicSpli::~BicSpli() 
{
  delete npoo;
}

static InitClass init("BicSpli", &BicSpli::Create, "Source/UserModules/BicSpli/");
