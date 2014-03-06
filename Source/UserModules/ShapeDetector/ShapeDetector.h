//
//	ShapeDetector.h		This file is a part of the IKAROS project
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


#ifndef ShapeDetector_
#define ShapeDetector_

#include "IKAROS.h"
#include <vector>

class ShapeDetector: public Module
{
 public:
  static Module * Create(Parameter * p) { return new ShapeDetector(p); }

  float * imageMetaData;
  float * needToLearn;

  float ** inputImage;
  float ** inputCenters;
  float ** outputImage;

  int sizeX;
  int sizeY;
  int boardSize;
  int dilationLoops;
  int numberOfImages;
  int numberOfRotations;
  float sizeXLarge;
  float sizeYLarge;
  float shapeRadius;
  //float pi;

  std::vector<std::vector<std::vector<std::vector<float>>>> referenceShapes; 

 ShapeDetector(Parameter * p) : Module(p) {}
  virtual ~ShapeDetector() {}

  void Init();
  void Tick();
  int Evaluate(int jIn, int kIn, int lIn, int iIn);
  void RecogniseShape();
  void LearnNewShape();

};

#endif
