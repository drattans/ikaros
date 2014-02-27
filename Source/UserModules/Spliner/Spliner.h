//
//	Spliner.h		This file is a part of the IKAROS project
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


#ifndef Spliner_
#define Spliner_

#include "IKAROS.h"
#include <vector> 

using namespace std;

typedef vector<double> VecDoub;

class Spliner: public Module
{
 public:
  static Module * Create(Parameter * p) { return new Spliner(p); }

  float * goalPosition;
  float * pushableData;
  float * mspec;
  float * newFingerInstructions;
  float * needNewShape;
  float * tapot;
  float * oldFingerInstructions;
  float * oldPushableData;
  float * cartesianFingerPosition;
  float * oldCauseEffect;
  float * causeEffect;
  float * temporaryGoal;

  float pi;
  float pushMarginal;
  float newPushAngle;
  float rotationAngle;

  int ticLagg;
  int ticLaggCounter;
  int ticLaggCounter2;
  int shapeErrors;
  int toleratedShapeErrors;

  bool justFoundNewAngle;
  bool differentSuccess;
  bool pushMode;
  bool pushComplete;
  bool justAtGoal;
  bool pushInitialised;
  bool protact;

  vector<VecDoub> xx;
  vector<VecDoub> yy;
  vector<vector<bool>> ok;

 Spliner(Parameter * p) : Module(p) {}
  virtual ~Spliner() {}

  int pointInPhaseSpace(float a, int b);

  float withinOneRevolution(float angtch);
  float findResultingAngle(float input);
  float findAngleToGoal(float input);

  void Manage(float fb[]);
  void anglesToCartesian(float& input1, float& input2);
  void prepareForNewShape();
  void Init();
  void Tick();
};

#endif

