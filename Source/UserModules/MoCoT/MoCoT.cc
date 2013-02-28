#include "MoCoT.h"
#include "math.h"

using namespace ikaros;

void
MoCoT::Init()
{
  //polar_in = GetInputArray("POLAR_IN");//(angle,radious)
  cart_in = GetInputArray("CART_IN");//(x,y)
  servo_out = GetOutputArray("SERVO_OUT");
  abcx = GetFloatValue("ABCX", 0.5f);//Arm base coordinate (x)
  abcy = GetFloatValue("ABCY", 1.f);//Arm base coordinate (y)
  y1 = 128.f; //Arm length (mm)
  y2 = 112.f; //Forearm length (mm)
  k = -7.f;//Arm base from table (mm)
  fe = 0.f;//Extention of finger (mm)
  h = 525.f;//Camera hight from surface (mm)
  pi = atan(1.f)*4.f;//Pi
}

void
MoCoT::Tick()
{
  float icc [2];// = {.0f,0.f};//Internal coordinates cartesian
  float icp [2];// = {.0f,0.f};//Internal coordinates polar
  /***
   * Cartesian to polar to servo
   ***/
  if(cart_in[0] <= 1 && cart_in[0] >= 0 && cart_in[1] <= 1 && cart_in[1] >= 0){
    icc[0] = (cart_in[0]-abcx)*2*h*tan(28.5*pi/180);
    icc[1] = (abcy-cart_in[1])*2*h*tan(21.5*pi/180);
    icp[0] = atan2(icc[1],icc[0]) * 180.f/pi - 90;
    icp[1] = sqrt(pow(icc[0],2)+pow(icc[1],2))-fe;
    if(icp[0] < 90 && icp[0] > -90 && icp[1] < 240 && icp[1] > 90){
      float x1 = pi/2 + asin(k/icp[1]) - acos((pow(y1,2) + pow(icp[1],2) + pow(k,2) - pow(y2,2))/(2*sqrt(pow(icp[1],2)+pow(k,2))*y1));
      float x2 = pi - acos((pow(y1,2) + pow(y2,2) - pow(icp[1],2) - pow(k,2))/(2*y1*y2));;
      servo_out[0] = 180.f + icp[0];//Shoulder (angle)
      servo_out[1] = 180.f - (x1 * 180.f/pi);//Shoulder (radious)
      servo_out[2] = 180.f - (x2 * 180.f/pi);//Elbow (radious)
      servo_out[3] = 270.f - ((x1+x2) * 180.f/pi);//Wrist
      //printf("iccx: %f, iccy: %f, icpf: %f, icpr: %f\n", icc[0], icc[1], icp[0], icp[1]);
      //printf("0: %f, 1: %f, 2: %f, 3: %f x1: %f x2: %f\n", servo_out[0], servo_out[1], servo_out[2], servo_out[3], x1, x2);
    }
    else{
      printf("Error (pc): %f, %f\n", icp[0], icp[1]);
    }
  }
  else{
    printf("Error (ci): %f, %f\n", cart_in[0], cart_in[1]);
  }
}

static InitClass init("MoCoT", &MoCoT::Create, "Source/UserModules/MoCoT/");


