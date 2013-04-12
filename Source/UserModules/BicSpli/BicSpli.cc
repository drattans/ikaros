#include "BicSpli.h"
#include "interp.h"
#include "math.h"
#include <iostream>
#include <fstream>
#include <ctime>

using namespace std;
using namespace ikaros;

void
BicSpli::Init()
{
  srand(std::time(NULL));
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
  just = false;
  first = false;
  pmode = false;//Push mode
  pdone = false;//Push done
  ddone = false;//Finished recently?
  initi = false;//Push initialised
  firstM = true;
  fipo = new float[2];
  fbo = new float[2];
  fb = new float[2];
  tg = new float[2];//Temporary goal
  radious = 30.f;//Good if it could find this by itself, but can be put to >21 for the time being
  pl = 35.f;//Push length
  te = 5.f;//Tolerated error
  tlcm = 5;//Tic-lagg-counter max
  tlc = tlcm;//Tic-lagg-counter
  tlc2 = tlcm;//Tic-lagg-counter
  na = (float)rand()/((float)RAND_MAX/(2*pi))-pi;//New angle, initiated as a random float from -pi to pi
  //printf("NA!!!!!: %f\n", na);
  ok.push_back(true);
}

void
BicSpli::Tick()
{
  tapo[0] = tapo[0] - abcx;
  tapo[1] = abcy - tapo[1];
  pupo[0] = pupo[0] - abcx;
  pupo[1] = abcy - pupo[1];
  float tpd = sqrt(pow((pupo[0]-tapo[0]), 2) + pow((pupo[1]-tapo[1]),2));//Target-pushable distance
  if(tpd < 0.03f){//If at goal
    //printf("Error (tpd): %f\n", tpd);
    printf("             ~*\\Done/*~             \n");
    ddone = true;
    //just = true;
    //pmode = false;
    pdone = false;
    el[0] = -40.f;
    //* Only once, but not first time
    if(first==true){
      first=false;
      tlc=tlcm;
      tpao=tpa;
      pdone = false;
      //float fbd = sqrt(pow((pupo[0]-tapo[0]), 2) + pow((pupo[1]-tapo[1]),2));
      float fbd = sqrt(pow((pupo[0]-pupoo[0]), 2) + pow((pupo[1]-pupoo[1]),2));
      fbo[0] = fb[0];
      fbo[1] = fb[1];
      if(fbd == 0){
	printf("Did not move (tpd): %f\n", fbd);
      }
      else if((pupoo[1]-pupo[1]) > 0){
	fb[0] = - acos((pupoo[0]-pupo[0])/fbd);
      }
      else{
	fb[0] = - acos(-(pupoo[0]-pupo[0])/fbd) + pi;
      }
      fb[1] = na;//Attack angle
      //fb[0] = -fb[0];
      if(abs(tpao-fb[0])>pi/10.f){
	int ttt=PosinX(tpao);
	if((tpao>0.9f*pi && fb[0]<-0.9f*pi) || (fb[0]>0.9f*pi && tpao<-0.9f*pi)){
	  printf("Helped\n");
	}
	else{
	  ok.at(ttt)=false;
	}
      }
      Manage(fb);
    }
    //*/
  }
  else if((pupo[1]-tapo[1]) > 0){
    tpa =  acos(-(pupo[0]-tapo[0])/tpd)-pi;
  }
  else{
    tpa =  acos((pupo[0]-tapo[0])/tpd);
  }
  //tpa=-tpa;
  //printf("Tpa: %f\n", tpa);
  if(tpa<-pi || tpa>pi){
    printf("Error (tpa): %f\n", tpa);
  }
  else if(tpd < 0.03f){
    //printf("Error (tpd): %f\n", tpd);
  }

  /***
   * The interessting part
   ***/
  else{
    ATC(pin);
    fipo[0] = pin[0];
    fipo[1] = pin[2];
    first = true;

    /***
     * Prepare for pushing
     ***/
    if(ddone==true){
      ddone = false;
    	int t=PosinX(tpa);
	if(ok.at(t) && xx.size()>1){
	  printf("Knw!~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	  Spline_interp *pushEffect = new Spline_interp(xx,yy);
	  na = pushEffect->interp(tpa);
	  //printf("M: Na: %f, Tpa: %f\n", na, tpa);
	  if(na<-pi || na>pi){
	    na = (float)rand()/((float)RAND_MAX/(2*pi))-pi;
	  }
	  just=true;
	}
	else{
	  printf("Rand!++++++++++++++++++++++++++++++++++++++++++++++++\n");
	  na = (float)rand()/((float)RAND_MAX/(2*pi))-pi;
	  just=true;
	}
    }

    /***
     * Evaluation
     ***/
    if(pdone==true){
      if(tlc>0){
	--tlc;
      }
      else{
	tlc=tlcm;
	tpao=tpa;
	pdone = false;
	//float fbd = sqrt(pow((pupo[0]-tapo[0]), 2) + pow((pupo[1]-tapo[1]),2));
	float fbd = sqrt(pow((pupo[0]-pupoo[0]), 2) + pow((pupo[1]-pupoo[1]),2));
	fbo[0] = fb[0];
	fbo[1] = fb[1];
	if(fbd == 0){
	  printf("Did not move (tpd): %f\n", fbd);
	}
	else if((pupoo[1]-pupo[1]) > 0){
	  fb[0] = - acos((pupoo[0]-pupo[0])/fbd);
	}
	else{
	  fb[0] = - acos(-(pupoo[0]-pupo[0])/fbd) + pi;
	}
	fb[1] = na;//Attack angle
	//fb[0] = -fb[0];
	if(abs(tpao-fb[0])>pi/10.f){
	  int ttt=PosinX(tpao);
	  if((tpao>0.9f*pi && fb[0]<-0.9f*pi) || (fb[0]>0.9f*pi && tpao<-0.9f*pi)){
	    printf("Helped\n");
	  }
	  else{
	    ok.at(ttt)=false;
	  }
	}
	Manage(fb);
	int t=PosinX(tpa);
	if(ok.at(t) && xx.size()>1){
	  printf("Knw!~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	  Spline_interp *pushEffect = new Spline_interp(xx,yy);
	  na = pushEffect->interp(tpa);
	  //printf("M: Na: %f, Tpa: %f\n", na, tpa);
	  if(na<-pi || na>pi){
	    na = (float)rand()/((float)RAND_MAX/(2*pi))-pi;
	  }
	  just=true;
	}
	else{
	  printf("Rand!++++++++++++++++++++++++++++++++++++++++++++++++\n");
	  na = (float)rand()/((float)RAND_MAX/(2*pi))-pi;
	  just=true;
	}
      }
    }

    /***
     * Going to attack position
     ***/
    else if((abs(npoo[0]-fipo[0])>te || abs(npoo[1]-fipo[1])>te || just==true) && initi==false){
      just=false;
      pmode = false;
      float ar1 = abs(npoo[0]-fipo[0]);//Distance to x
      float ar2 = abs(npoo[1]-fipo[1]);//Distance to y
      float f1 = ((radious*cos(na))/(2.f*h*tan(28.5*pi/180.f)));//Attack position, origo in pushable
      float f2 = ((radious*sin(na))/(2.f*h*tan(21.5*pi/180.f)));//Attack position, origo in pushable
      npo[0] = pupo[0] - f1 + abcx;//Attack position
      npo[1] = abcy - pupo[1] + f2;//Attack position
      npoo[0] = (npo[0] - abcx)*(2.f*h*tan(28.5*pi/180.f));
      npoo[1] = (abcy - npo[1])*(2.f*h*tan(21.5*pi/180.f));
    }

    /***
     * Pushing
     ***/
    else if(initi==false){
      pmode = true;
      if(tlc2>0){
	--tlc2;
      }
      else{
	tlc2=tlcm;
	float f1 = (((radious-pl)*cos(na))/(2*h*tan(28.5*pi/180)));//Finger target, origo in pushable
	float f2 = (((radious-pl)*sin(na))/(2*h*tan(21.5*pi/180)));//Finger target, origo in pushable
	tg[0] = pupo[0] - f1 + abcx;//Finger target
	tg[1] = abcy - pupo[1] - f2;//Finger target
	npo[0] = tg[0];
	npo[1] = tg[1];
	npoo[0] = (npo[0] - abcx)*(2*h*tan(28.5*pi/180));
	npoo[1] = (abcy - npo[1])*(2*h*tan(21.5*pi/180));
	printf("Na: %f, Tpa: %f\n", na, tpa);
	pupoo[0] = pupo[0];
	pupoo[1] = pupo[1];
	initi=true;
      }
    }
    else if((abs(npoo[0]-fipo[0])>te || abs(npoo[1]-fipo[1])>te)){
      npo[0] = tg[0];
      npo[1] = tg[1];
      npoo[0] = (npo[0] - abcx)*(2*h*tan(28.5*pi/180));
      npoo[1] = (abcy - npo[1])*(2*h*tan(21.5*pi/180));
    }
    else{
      initi = false;
      pdone = true;
      ddone = false;
    }

    if(pmode == true){
      el[0] = 0.f;
    }
    else{
      el[0] = -40.f;//-40.f
    }
  }
}

/************************************************************************************************
 *                                                                                              *
 *                                         Other stuff                                          *
 *                                                                                              *
 ************************************************************************************************/

void
BicSpli::Manage(float fb[])
{
  printf("Managed!\n");
  float teM = pi/30.f;//Tolerated error
  float nat;//New angle test
  /***
   * Add new data point?
   ***/
  if(xx.size()>1){
    Spline_interp *pushEffectManage = new Spline_interp(xx,yy);
    nat = pushEffectManage->interp(fb[0]);
    if(abs(nat-fb[1])>teM || ok.at(PosinX(fb[0]))==false){//Prediction wrong, time to learn
      int t=PosinX(fb[0]);
      if(fb[1]>pi || fb[1]<-pi){
	//printf("YY: %f, time: %i\n", fb[1], xx.size());
      }
      else if(fb[0]!=xx[t]){
	ok.erase(ok.begin()+t);
	ok.insert(ok.begin()+t, true);
	ok.insert(ok.begin()+t, true);
	xx.insert(xx.begin()+t, fb[0]);
	yy.insert(yy.begin()+t, fb[1]);
      }
      if(temPo.size()>0 && xx.size()>1){//Remove the random points (used for initiation)
	int tt=0;
	for(int i=0; i<xx.size(); ++i){
	  if(temPo.front()==xx[i]){
	    tt=i;
	    break;
	  }
	}
	ok.erase(ok.begin()+tt);
	ok.erase(ok.begin()+tt);
	ok.insert(ok.begin()+tt, true);
	xx.erase(xx.begin()+tt);
	yy.erase(yy.begin()+tt);
	if(temPo.front()==fb[0]){
	  ok.erase(ok.begin()+t);
	  ok.insert(ok.begin()+t, true);
	  ok.insert(ok.begin()+t, true);
	  xx.insert(xx.begin()+t, fb[0]);
	  yy.insert(yy.begin()+t, fb[1]);
	}
	temPo.erase(temPo.begin());
	//printf("Number of random points: %i\n", temPo.size());
      }
    }
    delete pushEffectManage;
  }
  else if(xx.size()<1){
    //if(firstM == true){
    //printf("Managed, in loop, %i!\n", xx.size());
    //temPo.push_back(fb[0]);
    ok.erase(ok.begin() + (ok.size()-1));
    ok.push_back(true);
    ok.push_back(true);
    xx.push_back(fb[0]);
    yy.push_back(fb[1]);
    //firstM=false;
  }
  else if(fb[0]!=xx[0]){
    //temPo.push_back(fb[0]);
    if(fb[0]<xx[0]){
      //xx.insert(0, fb[0]);
      //yy.insert(0, fb[1]);
      ok.erase(ok.begin());
      ok.insert(ok.begin(), true);
      ok.insert(ok.begin(), true);
      xx.insert(xx.begin(), fb[0]);
      yy.insert(yy.begin(), fb[1]);
      //printf("Managed,fb<xx, new first, in loop, %i!\n", xx.size());
    }
    else{
      ok.erase(ok.begin() + (ok.size()-1));
      ok.push_back(true);
      ok.push_back(true);
      xx.push_back(fb[0]);
      yy.push_back(fb[1]);
    }    
  }
  if(xx.size()>0){
    ofstream rdp;
    rdp.open ("measured.txt");
    for(int fl=0; fl<xx.size(); ++fl){
      rdp << xx[fl] << " " << yy[fl] << "\n";
    }
    rdp.close();
    if(xx.size()>1){
      Spline_interp *pushEffectManage = new Spline_interp(xx,yy);
      ofstream gtp;
      ofstream kel;
      gtp.open ("generated.txt");
      kel.open ("generatedF.txt");
      for(float fl=-pi; fl<pi; fl+=0.1){
	gtp << fl << " " << pushEffectManage->interp(fl) << "\n";
	int ttt=PosinX(fl);
	if(!ok.at(ttt)){
	  kel << fl << " " << pushEffectManage->interp(fl) << "\n";
	}
      }
      delete pushEffectManage;
      gtp.close();
      kel.close();
    }
  }
  printf("Knowledge: %i\n-----------------------------------------------------\n", xx.size());
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
  float k = 10.f + el[0];//Arm base from table (mm)
  float fe = 60.f;//Extention of finger (mm)
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

int
BicSpli::PosinX(float a){
  int t=xx.size();
  for(int i=0; i<xx.size(); ++i){
    if(a<xx[i]){
      t=i;
      break;
    }
  }
  return t;
}

BicSpli::~BicSpli() 
{
  delete npoo;
}

static InitClass init("BicSpli", &BicSpli::Create, "Source/UserModules/BicSpli/");
