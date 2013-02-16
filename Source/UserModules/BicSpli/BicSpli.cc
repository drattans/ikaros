#include "BicSpli.h"
#include "interp.h"
#include "math.h"

using namespace ikaros;

void
BicSpli::Init()
{
  tapo = GetInputArray("TAPO");//Position of target(x,y)
  pupo = GetInputArray("PUPO");//Position of pushable center (x,y)
  npo = GetOutputArray("NPO");//New finger position (x,y)
  pi = atan(1.f)*4.f;//Pi
}

void
BicSpli::Tick()
{
  //NYI: value, radious, ml (movement length), npo2
  float na;//New angle
  float tpa;//Target-pushable angle
  float tpd = sqrt(pow((pupo[0]-tapo[0]), 2) + pow((pupo[1]-tapo[1]),2));//Target-pushable distance
  if(tpd == 0){//If tapo == pupo -> pushable at target
    printf("Error (tpd): %f\n", tpd);
  }
  else if((pupo[1]-tapo[1]) > 0){
    tpa = acos((pupo[0]-tapo[0])/tpd)*180/pi;
  }
  else{
    tpa = 180-acos((pupo[0]-tapo[0])/tpd)*180/pi;
  }
  if(tpa<0 || tpa>180){
    printf("Error (tpa): %f\n", tpa);
  }
  else if(tpd == 0){
    printf("Error (tpd): %f\n", tpd);
  }
  else{//The interessting part
    int t=0;
    for(int i=0; i<xx.size; ++i){
      if(value[0]<xx[i]){
	t=i;
	break;
      }
    }
    xx.insert(t, value[0]);
    yy.insert(t, value[1]);
    Spline_interp pushEffect(xx, yy);
    na = pushEffect.interp(tpa);
    npo[0] = pupo[0] + radious*cos(na*pi/180);
    npo[1] = pupo[1] + radious*sin(na*pi/180);
    npo2[0] = pupo[0] + (radious-mn)*cos(na*pi/180);
    npo2[1] = pupo[1] + (radious-ml)*sin(na*pi/180);
  }
}



static InitClass init("BicSpli", &BicSpli::Create, "Source/UserModules/BicSpli/");


