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
  indir = GetInputArray("INDIR");//Shape direction
  ininf = GetInputArray("ININ");//Shape index
  npo = GetOutputArray("NPO");//New finger position (x,y)
  el = GetOutputArray("EL");//Elevation of finger
  nns = GetOutputArray("NNS");//Need new shape?
  norin = GetInputArray("NORIN");//Informationon number of rotations
  abcx = GetFloatValue("ABCX", 0.5f);//Arm base coordinate (x)
  abcy = GetFloatValue("ABCY", 1.f);//Arm base coordinate (y)
  noR = (int)norin[0];
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
  tapot = new float[2];
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
  vector<double> xx1;
  vector<double> yy1;
  vector<bool> ok1;
  ok1.push_back(true);
  xx.push_back(xx1);
  yy.push_back(yy1);
  ok.push_back(ok1);
  protact=true;
  if(protact){//If 0 is prototype
    vector<double> xx2;
    vector<double> yy2;
    vector<bool> ok2;
    ok2.push_back(true);
    xx.push_back(xx2);
    yy.push_back(yy2);
    ok.push_back(ok2);
  }
  rota = 2*pi/noR;
  ercou = 0;//Error counter
  ercouli = 5;//Errors before new shape is needed
}

void
BicSpli::Tick()
{
  noR = (int)norin[0];
  rota = 2*pi/noR;
  nns[0] = 0.f;
  inin = (int)ininf[0];
  if(protact){//If 0 is prototype
    inin+=1;
  }
  //shift2 = indir[0]*rota;
  //printf("Nor: %i, Rot: %f, In: %f\n", noR, rota, indir[0]);
  shift2 = 0;
  //shift3 = indir[0]*rota;
  shift3 = 0;
  tapot[0] = tapo[0] - abcx;//actual position before rotation
  tapot[1] = abcy - tapo[1];//actual position before rotation
  pupo[0] = pupo[0] - abcx;
  pupo[1] = abcy - pupo[1];
  tapo[0] = ((tapot[0] - pupo[0])*cos(shift3) - (tapot[1] - pupo[1])*sin(shift3)) + pupo[0];//Rotation
  tapo[1] = ((tapot[0] - pupo[0])*sin(shift3) + (tapot[1] - pupo[1])*cos(shift3)) + pupo[1];//Rotation
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
      float fbd = sqrt(pow((pupo[0]-pupoo[0]), 2) + pow((pupo[1]-pupoo[1]),2));
      fbo[0] = fb[0];
      fbo[1] = fb[1];
      if(fbd == 0){
	printf("Did not move (tpd): %f\n", fbd);
      }
      //else if((pupoo[1]-pupo[1]) > 0){
      //fb[0] = -acos((pupoo[0]-pupo[0])/fbd);
      //}
      //else{
      //fb[0] = -acos(-(pupoo[0]-pupo[0])/fbd) + pi;
      //}
      fb[1] = na;//Attack angle
      fb[0] = mtci(findTA(fbd)+shift2);
      if(abs(tpao-fb[0])>pi/10.f){//If result diveerges too much from anticipation
	int ttt=PosinX(tpao, inin);
	if((tpao>0.9f*pi && fb[0]<-0.9f*pi) || (fb[0]>0.9f*pi && tpao<-0.9f*pi)){
	  printf("Helped\n");
	}
	else{
	  ok[inin].at(ttt)=false;
	}
      }
      Manage(fb);
    }
    //*/
  }
  /*
  else if((pupo[1]-tapo[1]) > 0){
    tpa = mtci(acos(-(pupo[0]-tapo[0])/tpd)-pi+shift2);
  }
  else{
    tpa = mtci(acos((pupo[0]-tapo[0])/tpd)+shift2);
  }
  ///
  else if((pupo[0]-tapo[0]) > 0){
    tpa = mtci(acos((pupo[1]-tapo[1])/tpd)+shift2);
    //printf("Tpa: %f!\n", tpa);
  }
  else{
    tpa = mtci(-acos((pupo[1]-tapo[1])/tpd)+shift2);
    //printf("Tpa: %f?\n", tpa);
  }
  */
  tpa = findTPA(tpd);
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
      int t=PosinX(tpa, inin);
      if(ok[inin].at(t) && xx[inin].size()>1){
	printf("Knw!~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	Spline_interp *pushEffect = new Spline_interp(xx[inin],yy[inin]);
	na = mtci((pushEffect->interp(tpa))+shift2);
	//printf("M: Na: %f, Tpa: %f\n", na, tpa);
	//if(na<-pi || na>pi){
	//na = (float)rand()/((float)RAND_MAX/(2*pi))-pi;
	//}
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
	/*
	else if((pupoo[1]-pupo[1]) > 0){
	  fb[0] = - acos((pupoo[0]-pupo[0])/fbd);
	}
	else{
	  fb[0] = - acos(-(pupoo[0]-pupo[0])/fbd) + pi;
	}
	*/
	fb[1] = na;//Attack angle
	fb[0] = mtci(findTA(fbd)+shift2);
	if(abs(tpao-fb[0])>pi/10.f){
	  int ttt=PosinX(tpao, inin);
	  if((tpao>0.9f*pi && fb[0]<-0.9f*pi) || (fb[0]>0.9f*pi && tpao<-0.9f*pi)){
	    printf("Helped\n");
	  }
	  else{
	    ok[inin].at(ttt)=false;
	  }
	}
	Manage(fb);
	int t=PosinX(tpa, inin);
	if(ok[inin].at(t) && xx[inin].size()>1){
	  printf("Knw!~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	  Spline_interp *pushEffect = new Spline_interp(xx[inin],yy[inin]);
	  na = mtci((pushEffect->interp(tpa))+shift2);
	  //printf("M: Na: %f, Tpa: %f\n", na, tpa);
	  //if(na<-pi || na>pi){
	  //na = (float)rand()/((float)RAND_MAX/(2*pi))-pi;
	  //}
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
      float f1t = ((radious*cos(na))/(2.f*h*tan(28.5*pi/180.f)));//Attack position, origo in pushable
      float f2t = ((radious*sin(na))/(2.f*h*tan(21.5*pi/180.f)));//Attack position, origo in pushable
      float f1=f1t*cos(shift3)+f2t*sin(shift3);
      float f2=-f1t*sin(shift3)+f2t*cos(shift3);
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
	float f1t = (((radious-pl)*cos(na))/(2*h*tan(28.5*pi/180)));//Finger target, origo in pushable
	float f2t = (((radious-pl)*sin(na))/(2*h*tan(21.5*pi/180)));//Finger target, origo in pushable
	float f1=f1t*cos(shift3)+f2t*sin(shift3);
	float f2=-f1t*sin(shift3)+f2t*cos(shift3);
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

  //Need new shape?
  if(ercou>ercouli){
    ercou=0;
    NSH();
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
  if(xx[inin].size()>1){
    Spline_interp *pushEffectManage = new Spline_interp(xx[inin],yy[inin]);
    nat = mtci((pushEffectManage->interp(fb[0]))+shift2);
    double minXV = -pi;
    double maxXV = pi;
    if(PosinX(fb[0], inin)>0){
      minXV=xx[inin][PosinX(fb[0], inin)-1];
    }
    else if(PosinX(fb[0], inin)<xx[inin].size()){
      maxXV=xx[inin].at(PosinX(fb[0], inin));
    }
    if(abs(nat-fb[1])>teM || ok[inin].at(PosinX(fb[0], inin))==false || fabs(maxXV-minXV)>pi/10){//Prediction wrong, time to learn
      int t=PosinX(fb[0], inin);
      //if(abs(nat-fb[1])<=teM && ok[inin].at(PosinX(fb[0], inin))==true && fabs(maxXV-minXV)>pi/10){
      //printf("----------*'~-USEFULL-~'*----------\n");
      //}
      if(fb[1]>pi || fb[1]<-pi){
	//printf("YY: %f, time: %i\n", fb[1], xx[inin].size());
      }
      else if(fb[0]!=xx[inin][t]){
	ok[inin].erase(ok[inin].begin()+t);
	ok[inin].insert(ok[inin].begin()+t, true);
	ok[inin].insert(ok[inin].begin()+t, true);
	xx[inin].insert(xx[inin].begin()+t, fb[0]);
	yy[inin].insert(yy[inin].begin()+t, fb[1]);
      }
      /*
      if(temPo.size()>0 && xx[inin].size()>1){//Remove the random points (used for initiation)
	int tt=0;
	for(int i=0; i<xx[inin].size(); ++i){
	  if(temPo.front()==xx[inin][i]){
	    tt=i;
	    break;
	  }
	}
	ok[inin].erase(ok[inin].begin()+tt);
	ok[inin].erase(ok[inin].begin()+tt);
	ok[inin].insert(ok[inin].begin()+tt, true);
	xx[inin].erase(xx[inin].begin()+tt);
	yy[inin].erase(yy[inin].begin()+tt);
	if(temPo.front()==fb[0]){
	  ok[inin].erase(ok[inin].begin()+t);
	  ok[inin].insert(ok[inin].begin()+t, true);
	  ok[inin].insert(ok[inin].begin()+t, true);
	  xx[inin].insert(xx[inin].begin()+t, fb[0]);
	  yy[inin].insert(yy[inin].begin()+t, fb[1]);
	}
	temPo.erase(temPo.begin());
	//printf("Number of random points: %i\n", temPo.size());
      }
      */
    }
    delete pushEffectManage;
  }
  else if(xx[inin].size()<1){
    //if(firstM == true){
    //printf("Managed, in loop, %i!\n", xx[inin].size());
    //temPo.push_back(fb[0]);
    ok[inin].erase(ok[inin].begin() + (ok[inin].size()-1));
    ok[inin].push_back(true);
    ok[inin].push_back(true);
    xx[inin].push_back(fb[0]);
    yy[inin].push_back(fb[1]);
    //firstM=false;
  }
  else if(fb[0]!=xx[inin][0]){
    //temPo.push_back(fb[0]);
    if(fb[0]<xx[inin][0]){
      //xx[inin].insert(0, fb[0]);
      //yy[inin].insert(0, fb[1]);
      ok[inin].erase(ok[inin].begin());
      ok[inin].insert(ok[inin].begin(), true);
      ok[inin].insert(ok[inin].begin(), true);
      xx[inin].insert(xx[inin].begin(), fb[0]);
      yy[inin].insert(yy[inin].begin(), fb[1]);
      //printf("Managed,fb<xx[inin], new first, in loop, %i!\n", xx[inin].size());
    }
    else{
      ok[inin].erase(ok[inin].begin() + (ok[inin].size()-1));
      ok[inin].push_back(true);
      ok[inin].push_back(true);
      xx[inin].push_back(fb[0]);
      yy[inin].push_back(fb[1]);
    }    
  }
  if(xx[inin].size()>0){
    ofstream rdp;
    rdp.open ("measured.txt");
    for(int fl=0; fl<xx[inin].size(); ++fl){
      rdp << xx[inin][fl] << " " << yy[inin][fl] << "\n";
    }
    rdp.close();
    if(xx[inin].size()>1){
      Spline_interp *pushEffectManage = new Spline_interp(xx[inin],yy[inin]);
      ofstream gtp;
      ofstream kel;
      gtp.open ("generated.txt");
      kel.open ("generatedF.txt");
      for(float fl=-pi; fl<pi; fl+=0.1){
	gtp << fl << " " << pushEffectManage->interp(fl) << "\n";
	int ttt=PosinX(fl, inin);
	if(!ok[inin].at(ttt)){
	  kel << fl << " " << pushEffectManage->interp(fl) << "\n";
	}
      }
      delete pushEffectManage;
      gtp.close();
      kel.close();
    }
  }
  printf("Knowledge: %i\n-----------------------------------------------------\n", xx[inin].size());
  if(protact){//If 0 is prototype
    int prc=0;
    for(int pri=1; pri<xx.size(); ++pri){
      if(xx[pri].size()>1){
	Spline_interp *pushEffectManage = new Spline_interp(xx[pri],yy[pri]);
	nat = mtci((pushEffectManage->interp(fb[0]))+shift2);
	if(abs(nat-fb[1])<teM){//Prediction right
	  ++prc;
	}
	delete pushEffectManage;
      }
    }
    if((prc/(xx.size()-1))>0.8){//If it's a good point in at least 80% of the cases...
      int t=PosinX(fb[0], 0);
      if(fb[1]>pi || fb[1]<-pi){
	//printf("YY: %f, time: %i\n", fb[1], xx[inin].size());
      }
      else if(xx[0].size()==0){
	ok[0].erase(ok[0].begin()+t);
	ok[0].insert(ok[0].begin()+t, true);
	ok[0].insert(ok[0].begin()+t, true);
	xx[0].insert(xx[0].begin()+t, fb[0]);
	yy[0].insert(yy[0].begin()+t, fb[1]);
      }
      else if(fb[0]!=xx[0][t]){
	ok[0].erase(ok[0].begin()+t);
	ok[0].insert(ok[0].begin()+t, true);
	ok[0].insert(ok[0].begin()+t, true);
	xx[0].insert(xx[0].begin()+t, fb[0]);
	yy[0].insert(yy[0].begin()+t, fb[1]);
      }
    }
  }
  /*
    if(pruning condition){
    int oi;
    xx[inin].erase(oi);
    yy[inin].erase(oi);
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

float
BicSpli::mtci(float angtch)
{
  /*
    if(angtch>pi){
    angtch-=pi;
    return mtci(angtch);
    }
    else if(angtch<-pi){
    angtch+=pi;
    return mtci(angtch);
    }
    else{
    return angtch;
    }
  */
  //printf("In: %f, Out: %f\n", angtch, ((angtch/pi)-(int)(angtch/pi))*pi);
  if((int)(angtch/pi) % 2 == 0){
    return ((angtch/pi)-(int)(angtch/pi))*pi;
  }
  else if(angtch>0){
    return ((angtch/pi)-(int)(angtch/pi)-1)*pi;
  }
  else{
    return ((angtch/pi)-(int)(angtch/pi)+1)*pi;
  }
}

int
BicSpli::PosinX(float a, int b){
  int t=xx[b].size();
  for(int i=0; i<xx[b].size(); ++i){
    if(a<xx[b][i]){
      t=i;
      break;
    }
  }
  return t;
}

void
BicSpli::NSH()
{
  nns[0] = 1.f;
  vector<double> xx1=xx[0];
  vector<double> yy1=yy[0];
  vector<bool> ok1=ok[0];
  xx.push_back(xx1);
  yy.push_back(yy1);
  ok.push_back(ok1);
}

float
BicSpli::findTA(float fbd)
{
  if((pupoo[0]-pupo[0]) > 0){
    return acos((pupoo[1]-pupo[1])/fbd);//+pi/2;
  }
  else{
    return -(acos((pupoo[1]-pupo[1])/fbd));//+pi/2);
  }
}

float
BicSpli::findTPA(float fbd)
{
  if((pupo[0]-tapo[0]) > 0){
    return acos((pupo[1]-tapo[1])/fbd);//+pi/2;
  }
  else{
    return -(acos((pupo[1]-tapo[1])/fbd));//+pi/2);
  }
}
/*
  if(fbd == 0){
  printf("Did not move (tpd): %f\n", fbd);
  }
  else if((pupoo[1]-pupo[1]) > 0){
  fb[0] = -acos((pupoo[0]-pupo[0])/fbd);
  }
  else{
  fb[0] = -acos(-(pupoo[0]-pupo[0])/fbd) + pi;
  }

  else if((pupo[1]-tapo[1]) > 0){
  tpa = mtci(acos(-(pupo[0]-tapo[0])/tpd)-pi+shift2);
  }
  else{
  tpa = mtci(acos((pupo[0]-tapo[0])/tpd)+shift2);
  }
*/
BicSpli::~BicSpli() 
{
  delete npoo;
}

static InitClass init("BicSpli", &BicSpli::Create, "Source/UserModules/BicSpli/");
