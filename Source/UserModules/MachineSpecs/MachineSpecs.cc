//
//	MachineSpecs.cc		This file is a part of the IKAROS project
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


#include "MachineSpecs.h"

using namespace ikaros;
/***
 * The elements in "mspec" are the following;
 * 
 ***/
void
MachineSpecs::Init()
{
  jointPositionInput = GetInputArray("JOINT_POSITION");//In degrees, in order from the shoulder
  mspec = GetOutputArray("MACHINE_INFO");
  // *=temporarty value; will be calculated in "MachineSpecs::Tick()"
  // **=the unit vector (1,0) points with angle 0
  mspec[0] = 1.f;//Shoulder position, x (fraction of image)
  mspec[1] = 0.5f;//Shoulder position, y (fraction of image)
  mspec[2] = 10.f;//Shoulder position, z (mm)

  mspec[3] = 126.f;//Arm length (mm)
  mspec[4] = 112.f;//Forearm length (mm)
  mspec[5] = 60.f;//Hand length (mm)
  mspec[6] = 0.f;//Shoulder-to-finger length (mm)*

  mspec[7] = 0.75f;//Camera viewing angle, x (radians)
  mspec[8] = 0.99f;//Camera viewing angle, y (radians)
  mspec[9] = 525.f;//Camera position, z (mm)

  mspec[10] = 0.f;//Shoulder rotation (radians)*, **
  mspec[11] = 0.f;//Shoulder angle (radians)*
  mspec[12] = 0.f;//Elbow angle (radians)*
  mspec[13] = 0.f;//Wrist angle (radians)*

  mspec[14] = 0.f;//Finger position, x (fraction of image)*
  mspec[15] = 0.f;//Finger position, y (fraction of image)*
  //mspec[16] = 0.f;//Finger position, z (mm) (two states)

  pi = atan(1.f)*4.f;//Pi
}



void
MachineSpecs::Tick()
{
  //mspec[2] = 10.f+el[0];//Arm base from table (mm)

  mspec[10] = (jointPositionInput[0] + 90.f)*pi/180.f;//Shoulder rotation
  mspec[11] = (180.f - jointPositionInput[1])*pi/180.f;//Shoulder angle
  mspec[12] = (180.f - jointPositionInput[2])*pi/180.f;//Elbow angle
  mspec[13] = (jointPositionInput[3])*pi/180.f;//Wrist angle (probably needs to be adjusted by some angle)
  mspec[6] = sqrt((pow(mspec[3],2) + pow(mspec[4],2) - 2*mspec[3]*mspec[4]*cos(pi-mspec[12])) - pow(mspec[9],2)) + mspec[5];//Shoulder-to-finger length
  mspec[14] = mspec[0] + (mspec[6]*sin(mspec[10])/(2*tan(mspec[7]/2)*mspec[9]));//Finger position, x (fraction of image)
  mspec[15] = mspec[1] + (mspec[6]*cos(mspec[10])/(2*tan(mspec[8]/2)*mspec[9]));//Finger position, y (fraction of image)
}

static InitClass init("MachineSpecs", &MachineSpecs::Create, "Source/UserModules/MachineSpecs/");


