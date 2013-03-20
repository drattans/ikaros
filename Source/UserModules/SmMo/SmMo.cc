#include "SmMo.h"
#include "math.h"

using namespace ikaros;

void
SmMo::Init()
{
  //polar_in = GetInputArray("POLAR_IN");//(angle,radious)
  cart_in = GetInputArray("CART_IN");//(x,y)
  servo_out = GetOutputArray("SERVO_OUT");
  cart_out = GetOutputArray("CART_OUT");
  cupoa = GetInputArray("ANG_IN_CUR");//(a1, a2, a3, a4)
  abcx = GetFloatValue("ABCX", 0.5f);//Arm base coordinate (x)
  abcy = GetFloatValue("ABCY", 1.f);//Arm base coordinate (y)
  ml = GetFloatValue("segment_length", 10.f);//Move length
  el = GetInputArray("EL");//Elevation of finger
  smooth = GetBoolValue("smooth", false);
  y1 = 126.f; //Arm length (mm)
  y2 = 112.f; //Forearm length (mm)
  k = 10.f+el[0];//Arm base from table (mm)
  fe = 44.f;//Extention of finger (mm)
  h = 525.f;//Camera hight from surface (mm)
  pi = atan(1.f)*4.f;//Pi
}

void
SmMo::Tick()
{
  k = 10.f+el[0];
  float icc [2];// = {.0f,0.f};//Internal coordinates cartesian
  float icp [2];// = {.0f,0.f};//Internal coordinates polar
  /***
   * Cartesian to polar to servo
   ***/
  //printf("El: %f\n", el[0]);
  if(cart_in[0] <= 1 && cart_in[0] >= 0 && cart_in[1] <= 1 && cart_in[1] >= 0){
    icc[0] = (cart_in[0]-abcx)*2*h*tan(28.5*pi/180);
    icc[1] = (abcy-cart_in[1])*2*h*tan(21.5*pi/180);
    icp[0] = atan2(icc[1],icc[0]) * 180.f/pi - 90.f;
    icp[1] = sqrt(pow(icc[0],2)+pow(icc[1],2))-fe;
    cart_out[0] = abcx + icc[0]/(2*h*tan(28.5*pi/180));
    cart_out[1] = abcy - icc[1]/(2*h*tan(21.5*pi/180));
    //printf("Xg: %f, Yg: %f\n", cart_out[0], cart_out[1]);
    if(icp[0] < 90 && icp[0] > -90 && icp[1] < 240 && icp[1] > 90){
      if(smooth==false){
	float x1 = pi/2 + asin(k/icp[1]) - acos((pow(y1,2) + pow(icp[1],2) + pow(k,2) - pow(y2,2))/(2*sqrt(pow(icp[1],2)+pow(k,2))*y1));
	float x2 = pi - acos((pow(y1,2) + pow(y2,2) - pow(icp[1],2) - pow(k,2))/(2*y1*y2));
	servo_out[0] = 180.f + icp[0];//Shoulder (angle)
	servo_out[1] = 180.f - (x1 * 180.f/pi);//Shoulder (radious)
	servo_out[2] = 180.f - (x2 * 180.f/pi);//Elbow (radious)
	servo_out[3] = 270.f - ((x1+x2) * 180.f/pi);//Wrist
	//printf("iccx: %f, iccy: %f, icpf: %f, icpr: %f\n", icc[0], icc[1], icp[0], icp[1]);
	//printf("0: %f, 1: %f, 2: %f, 3: %f x1: %f x2: %f\n", servo_out[0], servo_out[1], servo_out[2], servo_out[3], x1, x2);
      }
      else{
	float cupo [4];
	float iccp [2];
	float iccc [2];
	cupo[0] = (cupoa[0] - 180.f)*pi/180.f;
	cupo[1] = (180.f - cupoa[1])*pi/180.f;
	cupo[2] = (180.f - cupoa[2])*pi/180.f;
	//cupo[3] = cupoa[3];
	iccp[0] = cupo[0]+pi/2;//pi/2 here
	iccp[1] = sqrt((pow(y1,2) + pow(y2,2) - 2*y1*y2*cos(pi-cupo[2])) - pow(k,2)) + fe;
	iccc[0] = iccp[1]*cos(iccp[0]);
	iccc[1] = iccp[1]*sin(iccp[0]);
	//printf("X: %f, Y: %f\n", cart_out[0], cart_out[1]);
	//printf("iccx: %f, iccy: %f, icpa: %f, icpr: %f\n", iccc[0], iccc[1], iccp[0], iccp[1]);
	//float ml = 30.f;//Move length
	//float na;//New angle
	float gca;//Goal-current angle
	float gcd = sqrt(pow((iccc[0]-icc[0]), 2) + pow((iccc[1]-icc[1]),2));//Goal-current distance
	if(gcd < ml/2){
	  cart_out[0] = abcx + iccc[0]/(2*h*tan(28.5*pi/180));
	  cart_out[1] = abcy - iccc[1]/(2*h*tan(21.5*pi/180));
	  //printf("Almost done, (gcd): %f\n", gcd);
	  float x1 = pi/2 + asin(k/icp[1]) - acos((pow(y1,2) + pow(icp[1],2) + pow(k,2) - pow(y2,2))/(2*sqrt(pow(icp[1],2)+pow(k,2))*y1));
	  float x2 = pi - acos((pow(y1,2) + pow(y2,2) - pow(icp[1],2) - pow(k,2))/(2*y1*y2));
	  servo_out[0] = 180.f + icp[0];//Shoulder (angle)
	  servo_out[1] = 180.f - (x1 * 180.f/pi);//Shoulder (radious)
	  servo_out[2] = 180.f - (x2 * 180.f/pi);//Elbow (radious)
	  servo_out[3] = 270.f - ((x1+x2) * 180.f/pi);//Wrist
	}
	else{
	  if((iccc[1]-icc[1]) < 0){
	    gca = acos(-(iccc[0]-icc[0])/gcd);
	    //printf("Gca1: %f\n", gca);
	  }
	  else{
	    gca = acos((iccc[0]-icc[0])/gcd)-pi;
	    //printf("Gca2: %f\n", gca);
	  }
	  if(gca<-pi || gca>pi){
	    printf("Error (gca): %f\n", gca);
	  }
	  else{
	    //cart_out[0] = 0.5f+(iccc[0] + ml*cos(gca))/(2*h*tan(28.5*pi/180));
	    //cart_out[1] = 1.f-(iccc[1] + ml*sin(gca))/(2*h*tan(21.5*pi/180));
	    float incc [2];
	    //printf("Part (gcd): %f\n%f, %f, %f\n", gcd, gca, ml*sin(gca), ml*cos(gca));
	    incc[0] = iccc[0] + ml*cos(gca);
	    incc[1] = iccc[1] + ml*sin(gca);
	    //printf("Xf: %f, Yf: %f\n", incc[0], incc[1]);
	    icp[0] = atan2(incc[1],incc[0]) * 180.f/pi - 90.f;
	    icp[1] = sqrt(pow(incc[0],2)+pow(incc[1],2))-fe;
	    cart_out[0] = abcx + incc[0]/(2*h*tan(28.5*pi/180));
	    cart_out[1] = abcy - incc[1]/(2*h*tan(21.5*pi/180));
	    //printf("Xfs: %f, Yfs: %f\n", cart_out[0], cart_out[1]);
	    //printf("Xfl: %f, Yfl: %f\n", cart_in[0], cart_in[1]);
	    if(icp[0] < 90 && icp[0] > -90 && icp[1] < 240 && icp[1] > 90){
	      float x1 = pi/2 + asin(k/icp[1]) - acos((pow(y1,2) + pow(icp[1],2) + pow(k,2) - pow(y2,2))/(2*sqrt(pow(icp[1],2)+pow(k,2))*y1));
	      float x2 = pi - acos((pow(y1,2) + pow(y2,2) - pow(icp[1],2) - pow(k,2))/(2*y1*y2));
	      servo_out[0] = 180.f + icp[0];//Shoulder (angle)
	      servo_out[1] = 180.f - (x1 * 180.f/pi);//Shoulder (radious)
	      servo_out[2] = 180.f - (x2 * 180.f/pi);//Elbow (radious)
	      servo_out[3] = 270.f - ((x1+x2) * 180.f/pi);//Wrist
	    }
	    else{
	      printf("Error (pc): %f, %f\n", icp[0], icp[1]);
	    }
	  }
	}
      }
    }
    else{
      printf("Error (pc): %f, %f\n", icp[0], icp[1]);
    }
  }
  else{
    printf("Error (ci): %f, %f\n", cart_in[0], cart_in[1]);
  }
}

static InitClass init("SmMo", &SmMo::Create, "Source/UserModules/SmMo/");


