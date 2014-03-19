//
//	Spliner.cc		This file is a part of the IKAROS project
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


#include "Spliner.h"
#include "interp.h"
#include "math.h"
#include <iostream>
#include <fstream>
#include <ctime>

using namespace std;
using namespace ikaros;
// tapo=>goalPosition
// pupo=>pushableData(2&3)
// indir=>pushableData(1)
// ininf=>pushableData(0)
// norin=>pushableData(4)
// noR=>pushableData(4)
// pin=>mspec(~10,11,12&13)
// abcx=>mspec(0)
// abcy=>mspec(1)
// h=>mspec(9)
// nns=>needNewShape
// npo=>newFingerInstructions(0&1)
// el=>newFingerInstructions(2)
// npoo=>oldFingerInstructions
// pupoo=>oldPushableData(2&3)
// first=>differentSuccess
void
Spliner::Init()
{
  srand(std::time(NULL));
  goalPosition = GetInputArray("GOAL_POSITION");//Position to which the pushable is supposed to go (x,y)
  pushableData = GetInputArray("PUSHABLE_DATA");//Information on the pushable (shape, orientation, center x, center y, posible rotations, radius)
  mspec = GetInputArray("MACHINE_INFO");
  // pin = GetInputArray("PIN");//Proprioseptic input (a1, a2, a3, a4)
  // indir = GetInputArray("INDIR");//Shape direction
  // ininf = GetInputArray("ININ");//Shape index
  newFingerInstructions = GetOutputArray("NEW_FINGER_INSTRUCTIONS");//New finger position (x,y,z) (z=elevation)
  //el = GetOutputArray("EL");//Elevation of finger
  needNewShape = GetOutputArray("NEED_NEW_SHAPE");//Need new shape?
  // norin = GetInputArray("NORIN");//Informationon number of rotations
  // abcx = GetFloatValue("ABCX", 0.5f);//Arm base coordinate (x)
  // abcy = GetFloatValue("ABCY", 1.f);//Arm base coordinate (y)
  // noR = (int)norin[0];
  // pi = atan(1.f)*4.f;//Pi
  // h = 525.f;//Camera hight from surface (mm)
  if(!oldFingerInstructions){//=if(oldFingerInstructions == NULL){
    oldFingerInstructions = new float[3];
    oldFingerInstructions[0] = 0.f;// newFingerInstructions of the last tick, x
    oldFingerInstructions[1] = 0.f;// newFingerInstructions of the last tick, y
    oldFingerInstructions[2] = 0.f;// newFingerInstructions of the last tick, z
  }
  if(!oldPushableData){//=if(oldPushableData == NULL){
    oldPushableData = new float[6];
    oldPushableData[0] = 0.f;// PushableData of the last tick, shape
    oldPushableData[1] = 0.f;// PushableData of the last tick, orientation
    oldPushableData[2] = 0.f;// PushableData of the last tick, x
    oldPushableData[3] = 0.f;// PushableData of the last tick, y
    oldPushableData[4] = 0.f;// PushableData of the last tick, posible rotations
    oldPushableData[5] = 0.f;// PushableData of the last tick, radius
  }
  //pupoo = new float[2];
  shapeIndex=0;
  justFoundNewAngle = false;//(just)
  differentSuccess = false;
  pushMode = false;//Push mode(pmode)
  pushComplete = false;//Push done(pdone)
  justAtGoal = false;//Finished recently?(ddone)
  pushInitialised = false;//Push initialised(initi)
  tapot = new float[2];//<=
  cartesianFingerPosition = new float[2];
  oldCauseEffect = new float[2];//<=(fbo)
  causeEffect = new float[2];//<=Direction to try/Resulting direction
  temporaryGoal = new float[2];//Temporary goal (tg)
  //radious = 30.f;//Good if it could find this by itself, but can be put to >21 for the time being
  pushMarginal=5.f;//Added to the radius of pushable when pushing
  //pushLength=pushableData[5];//now=radius of pushable 35.f;//Push length
  toleratedError = 5.f;//Tolerated error when evaluating if its done(te)
  ticLagg = 5;//Tic-lagg-counter max(tlcm)
  ticLaggCounter = ticLagg;//Tic-lagg-counter(tlc)
  ticLaggCounter2 = ticLagg;//Tic-lagg-counter(tlc2)
  newPushAngle = (float)rand()/((float)RAND_MAX/(2*pi))-pi;//New angle, initiated as a random float from -pi to pi
  vector<double> xx1;//<=
  vector<double> yy1;//<=
  vector<bool> ok1;//<=
  ok1.push_back(true);//<=
  xx.push_back(xx1);//<=
  yy.push_back(yy1);//<=
  ok.push_back(ok1);//<=
  protact=true;//<=Prototype?////////////////////////////
  if(protact){//If 0 is prototype//<=
    vector<double> xx2;//<=
    vector<double> yy2;//<=
    vector<bool> ok2;//<=
    ok2.push_back(true);//<=
    xx.push_back(xx2);//<=
    yy.push_back(yy2);//<=
    ok.push_back(ok2);//<=
  }//////////////////////////////////////////////////////
  rotationAngle = 2*pi/pushableData[4];//The size of a rotation. Division with zero?//<=
  shapeErrors = 0;//Error counter
  toleratedShapeErrors = 5;//Errors before new shape is needed
}

void
Spliner::Tick()
{
    /////////////////
   //  Preparing  //***********
  /////////////////

  rotationAngle = 2*pi/pushableData[4];
  needNewShape[0] = 0.f;
  shapeIndex=(int)pushableData[0];
  //inin = (int)ininf[0];//pushableData[0];
  //if(protact){//If 0 is prototype////////////////////////
  //  inin+=1;
  //}//////////////////////////////////////////////////////
  shift2 = 0;
  shift3 = 0;
  tapot[0] = goalPosition[0] - mspec[0];//actual position before rotation
  tapot[1] = mspec[1] - goalPosition[1];//actual position before rotation
  pushableData[2] = pushableData[2] - mspec[0];
  pushableData[3] = mspec[1] - pushableData[3];
  goalPosition[0] = ((tapot[0] - pushableData[2])*cos(shift3) - (tapot[1] - pushableData[3])*sin(shift3)) + pushableData[2];//Rotation
  goalPosition[1] = ((tapot[0] - pushableData[2])*sin(shift3) + (tapot[1] - pushableData[3])*cos(shift3)) + pushableData[3];//Rotation
  float distanceToGoal = sqrt(pow((pushableData[2]-goalPosition[0]), 2) + pow((pushableData[3]-goalPosition[1]),2));//Target-pushable distance

    //////////////////
   //  If at goal  //************************
  //////////////////

  if(distanceToGoal < 0.03f){//If at goal
    printf("             ~*\\Done/*~             \n");
    justAtGoal = true;//Last tick it was at the goal
    pushComplete = false;
    newFingerInstructions[2] = -40.f;
    if(differentSuccess==true){//If this is the first time arriving at the goal
      differentSuccess=false;
      ticLaggCounter=ticLagg;
      oldAngleToGoal=angleToGoal;
      float distanceMoved = sqrt(pow((pushableData[2]-oldPushableData[2]), 2) + pow((pushableData[3]-oldPushableData[3]),2));
      oldCauseEffect[0] = causeEffect[0];
      oldCauseEffect[1] = causeEffect[1];
      if(distanceMoved == 0){
	printf("Did not move (distanceToGoal): %f\n", distanceMoved);
      }
      causeEffect[1] = newPushAngle;//Attack angle
      causeEffect[0] = withinOneRevolution(findResultingAngle(distanceMoved)+shift2);//<=
      if(abs(oldAngleToGoal-causeEffect[0])>pi/10.f){//If result diveerges too much from anticipation
	int currentPoint=pointInPhaseSpace(oldAngleToGoal, pushableData[0]);//(PosinX=>pointInPhaseSpace)
	if((oldAngleToGoal>0.9f*pi && causeEffect[0]<-0.9f*pi) || (causeEffect[0]>0.9f*pi && oldAngleToGoal<-0.9f*pi)){
	  printf("Helped\n");
	}
	else{
	  ok[pushableData[0]].at(currentPoint)=false;//<=
	}
      }
      Manage(causeEffect);
    }
  }

    /////////////////
   //  Otherwise  //
  /////////////////

  angleToGoal = findAngleToGoal(distanceToGoal);
  if(distanceToGoal < 0.03f){//If at goal
  }
  else if(angleToGoal<-pi || angleToGoal>pi){//Some weird error
    printf("Error (angleToGoal): %f\n", angleToGoal);
  }
  else{//Something happens
    anglesToCartesian(cartesianFingerPosition[0], cartesianFingerPosition[1]);//<= (pin=mspec[10]-mspec[13])///ATC
    differentSuccess = true;

      /////////////////////////
     /* Prepare for pushing */
    /////////////////////////
    if(justAtGoal==true){
      justAtGoal = false;
      int currentPoint=pointInPhaseSpace(angleToGoal, pushableData[0]);
      if(ok[pushableData[0]].at(currentPoint) && xx[pushableData[0]].size()>1){
	printf("Deducing optimal strategy from earlier experiences...\n");
	Spline_interp *pushEffect = new Spline_interp(xx[pushableData[0]],yy[pushableData[0]]);//<=
	newPushAngle = withinOneRevolution((pushEffect->interp(angleToGoal))+shift2);//<=
	justFoundNewAngle=true;
      }
      else{
	newPushAngle = (float)rand()/((float)RAND_MAX/(2*pi))-pi;
	printf("Deducing opt... Hmm... Maybe %f?\n", newPushAngle);
	justFoundNewAngle=true;
      }
    }

    /***
     * Evaluation
     ***/
    if(pushComplete==true){
      if(ticLaggCounter>0){
	--ticLaggCounter;
      }
      else{
	ticLaggCounter=ticLagg;
	oldAngleToGoal=angleToGoal;
	pushComplete = false;
	float distanceMoved = sqrt(pow((pushableData[2]-oldPushableData[2]), 2) + pow((pushableData[3]-oldPushableData[3]),2));
	oldCauseEffect[0] = causeEffect[0];
	oldCauseEffect[1] = causeEffect[1];
	if(distanceMoved == 0){
	  printf("Did not move (Evaluation): %f\n", distanceMoved);
	}
	causeEffect[1] = newPushAngle;//Attack angle
	causeEffect[0] = withinOneRevolution(findResultingAngle(distanceMoved)+shift2);//<=
	if(abs(oldAngleToGoal-causeEffect[0])>pi/10.f){
	  int currentPoint=pointInPhaseSpace(oldAngleToGoal, pushableData[0]);
	  if((oldAngleToGoal>0.9f*pi && causeEffect[0]<-0.9f*pi) || (causeEffect[0]>0.9f*pi && oldAngleToGoal<-0.9f*pi)){
	    printf("Helped\n");
	  }
	  else{
	    ok[pushableData[0]].at(currentPoint)=false;//<=
	  }
	}
	Manage(causeEffect);
	int currentPoint=pointInPhaseSpace(angleToGoal, pushableData[0]);
	if(ok[pushableData[0]].at(currentPoint) && xx[pushableData[0]].size()>1){
	  printf("Deducing optimal strategy from earlier experiences...\n");
	  Spline_interp *pushEffect = new Spline_interp(xx[pushableData[0]],yy[pushableData[0]]);//<=
	  newPushAngle = withinOneRevolution((pushEffect->interp(angleToGoal))+shift2);//<=
	  justFoundNewAngle=true;
	}
	else{
	  newPushAngle = (float)rand()/((float)RAND_MAX/(2*pi))-pi;
	  printf("Deducing opt... Hmm... Maybe %f?\n", newPushAngle);
	  justFoundNewAngle=true;
	}
      }
    }

    /***
     * Going to attack position
     ***/
    else if((abs(oldFingerInstructions[0]-cartesianFingerPosition[0])>toleratedError || abs(oldFingerInstructions[1]-cartesianFingerPosition[1])>toleratedError || justFoundNewAngle==true) && pushInitialised==false){
      justFoundNewAngle=false;
      pushMode = false;
      //float ar1 = abs(oldFingerInstructions[0]-cartesianFingerPosition[0]);//Distance to x
      //float ar2 = abs(oldFingerInstructions[1]-cartesianFingerPosition[1]);//Distance to y
      float temporaryAngleRelativePushable1 = ((pushableData[5]*cos(newPushAngle))/(2*mspec[9]*tan(mspec[7]/2)));//Attack position, origo in pushable
      float temporaryAngleRelativePushable2 = ((pushableData[5]*sin(newPushAngle))/(2*mspec[9]*tan(mspec[8]/2)));//Attack position, origo in pushable
      float angleRelativePushable1=temporaryAngleRelativePushable1*cos(shift3)+temporaryAngleRelativePushable2*sin(shift3);
      float angleRelativePushable2=-temporaryAngleRelativePushable1*sin(shift3)+temporaryAngleRelativePushable2*cos(shift3);
      newFingerInstructions[0] = pushableData[2] - angleRelativePushable1 + mspec[0];//Attack position
      newFingerInstructions[1] = mspec[1] - pushableData[3] + angleRelativePushable2;//Attack position
      oldFingerInstructions[0] = (newFingerInstructions[0] - mspec[0])*(2*mspec[9]*tan(mspec[7]/2));
      oldFingerInstructions[1] = (mspec[1] - newFingerInstructions[1])*(2*mspec[9]*tan(mspec[8]/2));
      printf("Going: pD0: %f, aRP1: %f, mspec0: %f\n", pushableData[2], angleRelativePushable1, mspec[0]);
      printf("Going: Fi0: %f, Fi1: %f\n", newFingerInstructions[0], newFingerInstructions[1]);
    }

    /***
     * Pushing
     ***/
    else if(pushInitialised==false){
      pushMode = true;
      if(ticLaggCounter2>0){
	--ticLaggCounter2;
      }
      else{
	ticLaggCounter2=ticLagg;
	float temporaryAngleRelativePushable1 = (((pushableData[5]-pushableData[5])*cos(newPushAngle))/(2*mspec[9]*tan(mspec[7]/2)));//Finger target, origo in pushable
	float temporaryAngleRelativePushable2 = (((pushableData[5]-pushableData[5])*sin(newPushAngle))/(2*mspec[9]*tan(mspec[8]/2)));//Finger target, origo in pushable
	float angleRelativePushable1=temporaryAngleRelativePushable1*cos(shift3)+temporaryAngleRelativePushable2*sin(shift3);
	float angleRelativePushable2=-temporaryAngleRelativePushable1*sin(shift3)+temporaryAngleRelativePushable2*cos(shift3);
	temporaryGoal[0] = pushableData[2] - angleRelativePushable1 + mspec[0];//Finger target
	temporaryGoal[1] = mspec[1] - pushableData[3] - angleRelativePushable2;//Finger target
	newFingerInstructions[0] = temporaryGoal[0];
	newFingerInstructions[1] = temporaryGoal[1];
	oldFingerInstructions[0] = (newFingerInstructions[0] - mspec[0])*(2*mspec[9]*tan(mspec[7]/2));
	oldFingerInstructions[1] = (mspec[1] - newFingerInstructions[1])*(2*mspec[9]*tan(mspec[8]/2));
	printf("NewPushAngle: %f, AngleToGoal: %f\n", newPushAngle, angleToGoal);
	oldPushableData[2] = pushableData[2];
	oldPushableData[3] = pushableData[3];
	pushInitialised=true;
	printf("Punshing: Fi0: %f, Fi1: %f\n", newFingerInstructions[0], newFingerInstructions[1]);
      }
    }

    /***
     * Keep pushing towards the temporary goal
     ***/
    else if((abs(oldFingerInstructions[0]-cartesianFingerPosition[0])>toleratedError || abs(oldFingerInstructions[1]-cartesianFingerPosition[1])>toleratedError)){
      newFingerInstructions[0] = temporaryGoal[0];
      newFingerInstructions[1] = temporaryGoal[1];
      oldFingerInstructions[0] = (newFingerInstructions[0] - mspec[0])*(2*mspec[9]*tan(mspec[7]/2));
      oldFingerInstructions[1] = (mspec[1] - newFingerInstructions[1])*(2*mspec[9]*tan(mspec[8]/2));
      printf("Still punshing: Fi0: %f, Fi1: %f\n", newFingerInstructions[0], newFingerInstructions[1]);
    }

    /***
     * The push is complete
     ***/
    else{
      pushInitialised = false;
      pushComplete = true;
      justAtGoal = false;
    }

      ///////////////////
     /* Post-prepping */
    ///////////////////

    if(pushMode == true){
      newFingerInstructions[2] = 0.f;
    }
    else{
      newFingerInstructions[2] = -40.f;
    }
  }

  //Need new shape?//<=
  //if(ercou>ercouli){//<=
  //  ercou=0;//<=
  //  prepareForNewShape();//<=
  //}//<=
}

/************************************************************************************************
 *                                                                                              *
 *                                         Other stuff                                          *
 *                                                                                              *
 ************************************************************************************************/

void
Spliner::Manage(float fb[])//////////////////////////////////////////////////////
{
  printf("Managed!\n");
  float toleratedErrorManaged = pi/30.f;//Tolerated error
  float newAngleProbe;//New angle test
  /***
   * Add new data point?
   ***/
  if(xx[shapeIndex].size()>1){
    Spline_interp *pushEffectManage = new Spline_interp(xx[shapeIndex],yy[shapeIndex]);
    newAngleProbe = withinOneRevolution((pushEffectManage->interp(causeEffect[0]))+shift2);
    double minXvalue = -pi;
    double maxXvalue = pi;
    if(pointInPhaseSpace(causeEffect[0], shapeIndex)>0){
      minXvalue=xx[shapeIndex][pointInPhaseSpace(causeEffect[0], shapeIndex)-1];
    }
    else if(pointInPhaseSpace(causeEffect[0], shapeIndex)<xx[shapeIndex].size()){
      maxXvalue=xx[shapeIndex].at(pointInPhaseSpace(causeEffect[0], shapeIndex));
    }
    if(abs(newAngleProbe-causeEffect[1])>toleratedErrorManaged || ok[shapeIndex].at(pointInPhaseSpace(causeEffect[0], shapeIndex))==false || fabs(maxXvalue-minXvalue)>pi/10){//Prediction wrong, time to learn
      int currentPoint=pointInPhaseSpace(causeEffect[0], shapeIndex);
      if(causeEffect[1]>pi || causeEffect[1]<-pi){
      }
      else if(causeEffect[0]!=xx[shapeIndex][currentPoint]){
	ok[shapeIndex].erase(ok[shapeIndex].begin()+currentPoint);
	ok[shapeIndex].insert(ok[shapeIndex].begin()+currentPoint, true);
	ok[shapeIndex].insert(ok[shapeIndex].begin()+currentPoint, true);
	xx[shapeIndex].insert(xx[shapeIndex].begin()+currentPoint, causeEffect[0]);
	yy[shapeIndex].insert(yy[shapeIndex].begin()+currentPoint, causeEffect[1]);
      }
    }
    delete pushEffectManage;
  }
  /***
   * If it has encountered too few situations
   ***/
  else if(xx[shapeIndex].size()<1){
    ok[shapeIndex].erase(ok[shapeIndex].begin() + (ok[shapeIndex].size()-1));
    ok[shapeIndex].push_back(true);
    ok[shapeIndex].push_back(true);
    xx[shapeIndex].push_back(causeEffect[0]);
    yy[shapeIndex].push_back(causeEffect[1]);
  }
  /***
   * It knows one thing, and tries another
   ***/
  else if(causeEffect[0]!=xx[shapeIndex][0]){
    if(causeEffect[0]<xx[shapeIndex][0]){
      ok[shapeIndex].erase(ok[shapeIndex].begin());
      ok[shapeIndex].insert(ok[shapeIndex].begin(), true);
      ok[shapeIndex].insert(ok[shapeIndex].begin(), true);
      xx[shapeIndex].insert(xx[shapeIndex].begin(), causeEffect[0]);
      yy[shapeIndex].insert(yy[shapeIndex].begin(), causeEffect[1]);
    }
    else{
      ok[shapeIndex].erase(ok[shapeIndex].begin() + (ok[shapeIndex].size()-1));
      ok[shapeIndex].push_back(true);
      ok[shapeIndex].push_back(true);
      xx[shapeIndex].push_back(causeEffect[0]);
      yy[shapeIndex].push_back(causeEffect[1]);
    }    
  }
  /***
   * Printing knowledge to files
   ***/
  if(xx[shapeIndex].size()>0){
    ofstream rdp;
    rdp.open ("measured.txt");
    for(int fl=0; fl<xx[shapeIndex].size(); ++fl){
      rdp << xx[shapeIndex][fl] << " " << yy[shapeIndex][fl] << "\n";
    }
    rdp.close();
    if(xx[shapeIndex].size()>1){
      Spline_interp *pushEffectManage = new Spline_interp(xx[shapeIndex],yy[shapeIndex]);
      ofstream gtp;
      ofstream kel;
      gtp.open ("generated.txt");
      kel.open ("generatedF.txt");
      for(float fl=-pi; fl<pi; fl+=0.1){
	gtp << fl << " " << pushEffectManage->interp(fl) << "\n";
	int ttt=pointInPhaseSpace(fl, shapeIndex);
	if(!ok[shapeIndex].at(ttt)){
	  kel << fl << " " << pushEffectManage->interp(fl) << "\n";
	}
      }
      delete pushEffectManage;
      gtp.close();
      kel.close();
    }
  }
  printf("At the moment I remember %i occations!\n-----------------------------------------------------\n", xx[shapeIndex].size());
  if(protact){//If 0 is prototype
    int prc=0;
    for(int pri=1; pri<xx.size(); ++pri){
      if(xx[pri].size()>1){
	Spline_interp *pushEffectManage = new Spline_interp(xx[pri],yy[pri]);
	newAngleProbe = withinOneRevolution((pushEffectManage->interp(causeEffect[0]))+shift2);
	if(abs(newAngleProbe-causeEffect[1])<toleratedErrorManaged){//Prediction right
	  ++prc;
	}
	delete pushEffectManage;
      }
    }
    if((prc/(xx.size()-1))>0.8){//If it's a good point in at least 80% of the cases...
      int currentPoint=pointInPhaseSpace(causeEffect[0], 0);
      if(causeEffect[1]>pi || causeEffect[1]<-pi){
      }
      else if(xx[0].size()==0){
	ok[0].erase(ok[0].begin()+currentPoint);
	ok[0].insert(ok[0].begin()+currentPoint, true);
	ok[0].insert(ok[0].begin()+currentPoint, true);
	xx[0].insert(xx[0].begin()+currentPoint, causeEffect[0]);
	yy[0].insert(yy[0].begin()+currentPoint, causeEffect[1]);
      }
      else if(causeEffect[0]!=xx[0][currentPoint]){
	ok[0].erase(ok[0].begin()+currentPoint);
	ok[0].insert(ok[0].begin()+currentPoint, true);
	ok[0].insert(ok[0].begin()+currentPoint, true);
	xx[0].insert(xx[0].begin()+currentPoint, causeEffect[0]);
	yy[0].insert(yy[0].begin()+currentPoint, causeEffect[1]);
      }
    }
  }
}

void
Spliner::anglesToCartesian(float& input1, float& input2){//Converts the angles to cartesian coordinates
  float inRadians [2];
  inRadians[0] = mspec[10] - pi;
  inRadians[1] = pi - mspec[12];
  float temporaryAngle = (inRadians[0]+pi/2);
  float temporaryRadius = (sqrt((pow(mspec[3],2) + pow(mspec[4],2) - 2*mspec[3]*mspec[4]*cos(pi-inRadians[1])) - pow((mspec[2]-newFingerInstructions[2]),2)) + mspec[5]);
  input1 = temporaryRadius*cos(temporaryAngle);
  input2 = temporaryRadius*sin(temporaryAngle);
}

float
Spliner::withinOneRevolution(float angtch){//Removes excessive revolutions
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
Spliner::pointInPhaseSpace(float a, int b){//Find the fitting place
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
Spliner::prepareForNewShape(){//Need more work before calling
  needNewShape[0] = 1.f;
  vector<double> xx1=xx[0];
  vector<double> yy1=yy[0];
  vector<bool> ok1=ok[0];
  xx.push_back(xx1);
  yy.push_back(yy1);
  ok.push_back(ok1);
}

float
Spliner::findResultingAngle(float input){
  if((oldPushableData[2]-pushableData[2]) > 0){
    return acos((oldPushableData[3]-pushableData[3])/input);//+pi/2;
  }
  else{
    return -(acos((oldPushableData[3]-pushableData[3])/input));//+pi/2);
  }
}

float
Spliner::findAngleToGoal(float input){
  if((pushableData[2]-goalPosition[0]) > 0){
    return acos((pushableData[3]-goalPosition[1])/input);//+pi/2;
  }
  else{
    return -(acos((pushableData[3]-goalPosition[1])/input));//+pi/2);
  }
}

Spliner::~Spliner() 
{
  delete oldFingerInstructions;
}

static InitClass init("Spliner", &Spliner::Create, "Source/UserModules/Spliner/");

