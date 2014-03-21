//
//	ServoInstructor.cc		This file is a part of the IKAROS project
//
//    Copyright (C) 2013 <Author Name>
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    See http://www.ikaros-project.org/ for more information.
//

#include "ServoInstructor.h"
#include "math.h"

using namespace ikaros;

void
ServoInstructor::Init()
{
  mspec = GetInputArray("MACHINE_INFO");
  newFingerInstructions = GetInputArray("NEW_FINGER_INSTRUCTIONS");//New finger position (x,y,z) (z=elevation)
  //cart_in = GetInputArray("CART_IN");//(x,y)
  servoNext = GetOutputArray("SERVO_NEXT");
  cartesianOutput = GetOutputArray("CARTESIAN_OUTPUT");
  cartesianOutputPlot = GetOutputArray("CARTESIAN_OUTPUT_PLOT");
  servoPresent = GetInputArray("SERVO_PRESENT");//(a1, a2, a3, a4)
  //mspec[0]//abcx = GetFloatValue("ABCX", 0.5f);//Arm base coordinate (x)
  //mspec[1]//abcy = GetFloatValue("ABCY", 1.f);//Arm base coordinate (y)
  segmentLength = GetFloatValue("SEGMENT_LENGTH", 10.f);//Move length
  //newFingerInstructions[2]//el = GetInputArray("EL");//Elevation of finger
  smooth = GetBoolValue("SMOOTH", true);
  //mspec[3]//y1 = 126.f; //Arm length (mm)
  //mspec[4]//y2 = 112.f; //Forearm length (mm)
  //zShift//mspec[2]-newFingerInstructions[2]//k = 10.f+el[0];//Arm base from table (mm)
  //mspec[5]//fe = 60.f;//Extention of finger (mm)
  //mspec[9]//h = 525.f;//Camera hight from surface (mm)
  //pi = atan(1.f)*4.f;//Pi
}

void
ServoInstructor::Tick()
{
  //k = 10.f+el[0];
  //newFingerInstructions[0]=0.5;
  //newFingerInstructions[1]=0.5;
  //newFingerInstructions[2]=0.0;
  float zShift = mspec[2]+newFingerInstructions[2];//Arm base from table (mm)
  float temporaryCartesian [2];// = {.0f,0.f};//Internal coordinates cartesian
  float temporaryPolar [2];// = {.0f,0.f};//Internal coordinates polar
  /***
   * Cartesian to polar to servo
   ***/
  if(newFingerInstructions[0] <= 1 && newFingerInstructions[0] >= 0 && newFingerInstructions[1] <= 1 && newFingerInstructions[1] >= 0){
    temporaryCartesian[0] = (mspec[0]-newFingerInstructions[0])*2*mspec[9]*tan(mspec[7]/2);
    temporaryCartesian[1] = (newFingerInstructions[1]-mspec[1])*2*mspec[9]*tan(mspec[8]/2);
    temporaryPolar[0] = atan2(temporaryCartesian[0],temporaryCartesian[1]) - pi/2;
    temporaryPolar[1] = sqrt(pow(temporaryCartesian[0],2)+pow(temporaryCartesian[1],2))-mspec[5];
    cartesianOutput[0] = mspec[0] - temporaryCartesian[0]/(2*mspec[9]*tan(mspec[7]/2));
    cartesianOutput[1] = mspec[1] + temporaryCartesian[1]/(2*mspec[9]*tan(mspec[8]/2));
    cartesianOutputPlot[0]=cartesianOutput[1];
    cartesianOutputPlot[1]=cartesianOutput[0];
    //printf("Fi0: %f, Fi1: %f\n", newFingerInstructions[0], newFingerInstructions[1]);
    //printf("Xg: %f, Yg: %f\n", temporaryCartesian[0], temporaryCartesian[1]);
    //printf("Xg: %f, Yg: %f\n", cartesianOutput[0], cartesianOutput[1]);
    //printf("Tp: %f, Tp: %f\n", temporaryPolar[0], temporaryPolar[1]);

    if(temporaryPolar[0] < pi/2 && temporaryPolar[0] > -pi/2 && temporaryPolar[1] < 240 && temporaryPolar[1] > 90){

      if(smooth==false){
	float x1 = pi/2 + asin(zShift/temporaryPolar[1]) - acos((pow(mspec[3],2) + pow(temporaryPolar[1],2) + pow(zShift,2) - pow(mspec[4],2))/(2*sqrt(pow(temporaryPolar[1],2)+pow(zShift,2))*mspec[3]));
	float x2 = pi - acos((pow(mspec[3],2) + pow(mspec[4],2) - pow(temporaryPolar[1],2) - pow(zShift,2))/(2*mspec[3]*mspec[4]));
	servoNext[0] = pi + temporaryPolar[0];//Shoulder (angle)
	servoNext[1] = pi - x1;//Shoulder (radius)
	servoNext[2] = pi - x2;//Elbow (radius)
	servoNext[3] = 3*pi/2 - (x1+x2);//Wrist
	//printf("temporaryCartesianx: %f, temporaryCartesiany: %f, temporaryPolarf: %f, temporaryPolarr: %f\n", temporaryCartesian[0], temporaryCartesian[1], temporaryPolar[0], temporaryPolar[1]);
	//printf("0: %f, 1: %f, 2: %f, 3: %f x1: %f x2: %f\n", servoNext[0], servoNext[1], servoNext[2], servoNext[3], x1, x2);
      }

      else{
	//printf("Sp: %f, Sp: %f\n", servoPresent[0], servoPresent[1]);
	float temporaryServoInstructions [4];
	float temporaryPolarSegment [2];
	float temporaryCartesianSegment [2];
	temporaryServoInstructions[0] = servoPresent[0];//servoPresent[0] - pi;
	temporaryServoInstructions[1] = servoPresent[1];//pi - servoPresent[1];
	temporaryServoInstructions[2] = servoPresent[2];//pi - servoPresent[2];
	temporaryPolarSegment[0] = temporaryServoInstructions[0]-pi*3/2;
	temporaryPolarSegment[1] = sqrt((pow(mspec[3],2) + pow(mspec[4],2) - 2*mspec[3]*mspec[4]*cos(pi+temporaryServoInstructions[2])) - pow(zShift,2)) + mspec[5];
	temporaryCartesianSegment[0] = -temporaryPolarSegment[1]*cos(temporaryPolarSegment[0]);
	temporaryCartesianSegment[1] = -temporaryPolarSegment[1]*sin(temporaryPolarSegment[0]);
	float angleToGoal;
	float distanceToGoal = sqrt(pow((temporaryCartesianSegment[0]-temporaryCartesian[0]), 2) + pow((temporaryCartesianSegment[1]-temporaryCartesian[1]),2));
	printf("Xg: %f, Yg: %f\n", temporaryPolarSegment[0], temporaryPolarSegment[1]);
	printf("Xg: %f, Yg: %f\n", temporaryCartesianSegment[0], temporaryCartesianSegment[1]);
	printf("Xg: %f, Yg: %f\n", temporaryCartesian[0], temporaryCartesian[1]);
	printf("DTG: %f\n", distanceToGoal);

	if(distanceToGoal < segmentLength/2){
	  //cartesianOutput[0] = mspec[0] - temporaryCartesianSegment[0]/(2*mspec[9]*tan(mspec[7]/2));
	  //cartesianOutput[1] = mspec[1] + temporaryCartesianSegment[1]/(2*mspec[9]*tan(mspec[8]/2));
	  float x1 = pi/2 + asin(zShift/temporaryPolar[1]) - acos((pow(mspec[3],2) + pow(temporaryPolar[1],2) + pow(zShift,2) - pow(mspec[4],2))/(2*sqrt(pow(temporaryPolar[1],2)+pow(zShift,2))*mspec[3]));
	  float x2 = pi - acos((pow(mspec[3],2) + pow(mspec[4],2) - pow(temporaryPolar[1],2) - pow(zShift,2))/(2*mspec[3]*mspec[4]));
	  servoNext[0] = pi + temporaryPolar[0];//Shoulder (angle)
	  servoNext[1] = pi - x1;//Shoulder (radius)
	  servoNext[2] = pi - x2;//Elbow (radius)
	  servoNext[3] = 3*pi/2 -(x1+x2);//Wrist
	}

	else{
	  if((temporaryCartesianSegment[1]-temporaryCartesian[1]) < 0){
	    angleToGoal = acos(-(temporaryCartesianSegment[0]-temporaryCartesian[0])/distanceToGoal);
	    //printf("AngleToGoal1: %f\n", angleToGoal);
	  }
	  else{
	    angleToGoal = acos((temporaryCartesianSegment[0]-temporaryCartesian[0])/distanceToGoal)-pi;
	    //printf("AngleToGoal2: %f\n", angleToGoal);
	  }

	  if(angleToGoal<-pi || angleToGoal>pi){
	    printf("Error (angleToGoal): %f\n", angleToGoal);
	  }
	  else{
	    float temporaryHolder [2];
	    //printf("Part (distanceToGoal): %f\n%f, %f, %f\n", distanceToGoal, angleToGoal, segmentLength*sin(angleToGoal), segmentLength*cos(angleToGoal));
	    temporaryHolder[0] = temporaryCartesianSegment[0] + segmentLength*cos(angleToGoal);
	    temporaryHolder[1] = temporaryCartesianSegment[1] + segmentLength*sin(angleToGoal);
	    temporaryPolar[0] = atan2(temporaryHolder[1],temporaryHolder[0]) - pi/2;
	    temporaryPolar[1] = sqrt(pow(temporaryHolder[0],2)+pow(temporaryHolder[1],2))-mspec[5];
	    cartesianOutput[0] = mspec[0] - temporaryHolder[0]/(2*mspec[9]*tan(mspec[7]/2));
	    cartesianOutput[1] = mspec[1] + temporaryHolder[1]/(2*mspec[9]*tan(mspec[8]/2));
	    if(temporaryPolar[0] < pi/2 && temporaryPolar[0] > -pi/2 && temporaryPolar[1] < 240 && temporaryPolar[1] > 90){
	      float x1 = pi/2 + asin(zShift/temporaryPolar[1]) - acos((pow(mspec[3],2) + pow(temporaryPolar[1],2) + pow(zShift,2) - pow(mspec[4],2))/(2*sqrt(pow(temporaryPolar[1],2)+pow(zShift,2))*mspec[3]));
	      float x2 = pi - acos((pow(mspec[3],2) + pow(mspec[4],2) - pow(temporaryPolar[1],2) - pow(zShift,2))/(2*mspec[3]*mspec[4]));
	      servoNext[0] = pi + temporaryPolar[0];//Shoulder (angle)
	      servoNext[1] = pi - x1;//Shoulder (radius)
	      servoNext[2] = pi - x2;//Elbow (radius)
	      servoNext[3] = 3*pi/4 -(x1+x2);//Wrist
	    }
	    else{
	      printf("Error (pc1): %f, %f\n", temporaryPolar[0], temporaryPolar[1]);
	    }
	  }
	}
      }
    }
    else{
      printf("Error (pc2): %f, %f\n", temporaryPolar[0], temporaryPolar[1]);
    }
  }
  else{
    printf("Error (ci): %f, %f\n", newFingerInstructions[0], newFingerInstructions[1]);
  }
}

static InitClass init("ServoInstructor", &ServoInstructor::Create, "Source/UserModules/ServoInstructor/");


