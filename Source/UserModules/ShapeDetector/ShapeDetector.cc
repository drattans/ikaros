//
//	ShapeDetector.cc		This file is a part of the IKAROS project
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

#include "ShapeDetector.h"
#include "math.h"
#include <vector>

using namespace ikaros;
using namespace std;

void
ShapeDetector::Init()
{
  inputImage=GetInputMatrix("INPUT_IMAGE");//Input image
  //sizeX=100;
  //sizeY=100;
  sizeX=GetInputSizeX("INPUT_IMAGE");//Size of the small images, x, should be 100
  sizeY=GetInputSizeY("INPUT_IMAGE");//Size of the small images, y, should be 100
  dilationLoops=GetIntValue("DILATION_LOOPS", 1);//Number of dilation loops
  needToLearn=GetInputArray("NEED_TO_LEARN");//Need to learn new shape
  numberOfImages=GetIntValue("NUMBER_OF_IMAGES", 1);//Number of images as input
  inputCenters=GetInputMatrix("INPUT_CENTERS");//Coordinates to the centers of the small images in the large image ((x1,y1), (x2,y2), ...)
  numberOfRotations=GetIntValue("NUMBER_OF_ROTATIONS", 1);//Total number of rotated versions of each image

  boardSize=25;//Size of black board around the input image in memory
  sizeXLarge=480.f;//Size of the original image, x
  sizeYLarge=640.f;//Size of the original image, y
  //pi=atan(1.f)*4.f;//Pi

  needToLearn[0]=1.f;

  outputImage=GetOutputMatrix("OUTPUT_IMAGE");//Output image
  imageMetaData=GetOutputArray("IMAGE_META_DATA");//Information on shape index, orientation, etc. If more images exists, this should be a matrix
  imageMetaData[0]=0;//shapeIndex;
  imageMetaData[1]=0;//orientationIndex;
  imageMetaData[2]=inputCenters[0][1]*sizeXLarge;//newCenter[0];
  imageMetaData[3]=inputCenters[0][0]*sizeYLarge;//newCenter[1];
  imageMetaData[4]=numberOfRotations;
  imageMetaData[5]=0;//shapeRadius;
}



void
ShapeDetector::Tick()
{
  for(int i=0; i<numberOfImages; ++i){//Changing coordinates from fraction of image size to pixel values
    float temp=inputCenters[i][1]*sizeXLarge;
    inputCenters[i][1]=inputCenters[i][0]*sizeYLarge;
    inputCenters[i][0]=temp;//inputCenters[i][1]*sizeYLarge;
    //printf("IcSx: %f, IcSy: %f\n", inputCenters[i][0], inputCenters[i][1]);
  }

  /**
   * Dilation loop, filling gaps in input
   **/
  if(dilationLoops>0){
    for(int i=0; i<numberOfImages; ++i){
      vector<vector<float>> inputImageTemporary;//A temporary matrix for storing a modified image
      inputImageTemporary.resize(sizeX);
      for(int j=0; j<sizeX; ++j){
	inputImageTemporary[j].resize(sizeY);
      }
      for(int k=0; k<dilationLoops; ++k){
	for (int l=1; l<sizeX-1; l++){
	  for (int m=1; m<sizeY-1; m++){
	    float t = inputImage[l][m];
	    for (int lt=-1; lt<2; lt++){
	      for (int it=-1; it<2; it++){
		if (inputImage[l+lt][m+it] > t){
		  t = inputImage[l+lt][m+it];
		}
	      }
	    }
	    inputImageTemporary[l-1][m-1] = t;
	  }
	}
	for (int l=0; l<sizeX; l++){
	  for (int m=0; m<sizeY; m++){
	    inputImage[l][m]=inputImageTemporary[l][m];
	  }
	}
      }
    }
  }

  /**
   * Learning and recognising shapes
   **/
  //Should be a for-loop is there are more than one input image
  if(needToLearn[0]>0.5f || referenceShapes.size()==0){
    float averageIntensity=0;
    for(int i=0; i<sizeX; ++i){
      for(int j=0; j<sizeY; ++j){
	averageIntensity+=inputImage[i][j];//This sum givs the total intensity
      }
    }
    averageIntensity/=sizeX*sizeY;//Dividing with the size gives an average
    if(averageIntensity>0.05){//Don't evaluate empty images
      LearnNewShape();
    }
  }
  if(referenceShapes.size()>0){
    RecogniseShape();
    printf("1: IcSx: %f, IcSy: %f\n", imageMetaData[2], imageMetaData[3]);
    imageMetaData[2]-=boardSize;//Should be in a loop
    imageMetaData[3]-=boardSize;//Should be in a loop
    printf("1: IcSx: %f, IcSy: %f\n", imageMetaData[2], imageMetaData[3]);
    imageMetaData[2]/=sizeXLarge;//Should be in a loop
    imageMetaData[3]/=sizeYLarge;//Should be in a loop
    printf("1: IcSx: %d, IcSy: %d\n", boardSize, boardSize);
    printf("1: IcSx: %f, IcSy: %f\n", sizeXLarge, sizeYLarge);
    printf("1: IcSx: %f, IcSy: %f\n", imageMetaData[2], imageMetaData[3]);
  }
  else{
    imageMetaData[2]=inputCenters[0][0]/sizeXLarge;//Should be in a loop
    imageMetaData[3]=inputCenters[0][1]/sizeYLarge;//Should be in a loop
    printf("2: IcSx: %f, IcSy: %f\n", imageMetaData[2], imageMetaData[3]);
  }
  //Is largest radius possible to find?
  //Yes, using information from the croping process when the shape was created
}

void
ShapeDetector::RecogniseShape()
{
  int maximumAgreement=0;
  printf("1");
  for(int i=0; i<referenceShapes.size(); ++i){//For each shape (using brute force)
    int ko;
    int jo;
    int lo=0;
    for(int l=0; l<numberOfRotations; ++l){//For each orientation
      for(int j=-10-boardSize; j<10-boardSize; ++j){//For each x-position:
	for(int k=-10-boardSize; k<10-boardSize; ++k){//For each y-position:
	  int agreement=0;
	  agreement=Evaluate(j, k, l, i);//Check similarity
	  if(agreement>maximumAgreement){//If it's the best agreement so far
	    maximumAgreement=agreement;
	    imageMetaData[0]=i;//Shape index
	    imageMetaData[1]=l;//Orientation index
	    imageMetaData[2]=inputCenters[0][0]+(k+boardSize);//New center in x
	    imageMetaData[3]=inputCenters[0][1]+(j+boardSize);//New center in y
	    //imageMetaData[4]=numberOfRotations;//referenceShapes[i].size();
	    imageMetaData[5]=shapeRadius;//Largest radius;
	    ko=k;
	    jo=j;
	    lo=l;
	    //printf("IcSx: %f, IcSy: %f\n", inputCenters[0][0], inputCenters[0][1]);
	    //printf("IcSx: %d, IcSy: %d\n", (k+boardSize), (j+boardSize));
	    printf("3: IcSx: %f, IcSy: %f\n", imageMetaData[2], imageMetaData[3]);
	  }
	}
      }
    }
    imageMetaData[0]=0;//Shape index
    imageMetaData[1]=0;//Orientation index
    imageMetaData[2]=inputCenters[0][0]+(boardSize);//New center in x
    imageMetaData[3]=inputCenters[0][1]+(boardSize);//New center in y
    //imageMetaData[4]=numberOfRotations;//referenceShapes[i].size();
    imageMetaData[5]=50;//Largest radius;
    printf("4:IcSx: %f, IcSy: %f\n", imageMetaData[2], imageMetaData[3]);
    //To be able to inspect how good the estimation is, the input and the guess is
    //merged.
    int l=lo;
    for(int m=0; m<sizeX+2*boardSize; ++m){
      for(int n=0; n<sizeY+2*boardSize; ++n){
	if(jo+m>=0 && sizeX>jo+m && ko+n>=0 && sizeY>ko+n){//>=0
	  outputImage[m][n]=(inputImage[jo+m][ko+n]+referenceShapes[i][l][m][n])/2;
	}
      }
    }
    //printf("IcSx: %i, IcSy: %i\n", sizeX+2*boardSize, sizeY+2*boardSize);
  }
}

//Matching active pixels=>+1, other=>0
int
ShapeDetector::Evaluate(int jIn, int kIn, int lIn, int iIn)
{
  int agreementTemporary=0;
  for(int m=0; m<sizeX+2*boardSize; ++m){
    for(int n=0; n<sizeY+2*boardSize; ++n){
      if(jIn+m>=0 && sizeX>jIn+m && kIn+n>=0 && sizeY>kIn+n){//>=0
        if(inputImage[jIn+m][kIn+n]>0.5 && referenceShapes[iIn][lIn][m][n]>0.5){
	  ++agreementTemporary;
        }
      }
    }
  }
  return agreementTemporary;
}

void
ShapeDetector::LearnNewShape()
{
  /**
   * Crop the image
   **/
  vector<vector<float>> croppedCentredBackground;
  croppedCentredBackground.resize(sizeX);
  for(int imr=0; imr<sizeX; ++imr){
    croppedCentredBackground[imr].resize(sizeY);
  }
  //Set all pixels in background to 0
  for(int imr=0; imr<sizeX; ++imr){
    for(int jmr=0; jmr<sizeY; ++jmr){
      croppedCentredBackground[imr][jmr]=0;
    }
  }
  int cropEdge [4]={0,0,sizeY,0};//Edges of croped image(xFirst,xLast,yFirst,yLast)
  //for all collumns:
  //Look for first and last non-empty collumn
  for(int ics=0; ics<sizeX; ++ics){
    //for all row:
    //Look for first and last non-empty row
    for(int jcs=0; jcs<sizeY; ++jcs){
      if(inputImage[ics][jcs]>0.5){
	if(cropEdge[0]==0){
	  cropEdge[0]=ics;
	}
	else{
	  cropEdge[1]=ics;
	}
	if(cropEdge[2]>jcs){
	  cropEdge[2]=jcs;
	}
	if(cropEdge[3]<jcs){
	  cropEdge[3]=jcs;
	}
      } 
    }
  }
  //Add the cropped part to the center of a vectorvector
  int shapeLengthX = cropEdge[1]-cropEdge[0];//Largest distance of shape in x
  int shapeLengthY = cropEdge[3]-cropEdge[2];//Largest distance of shape in y
  shapeRadius=(ceil(sqrt(pow((shapeLengthX/2),2.0)+pow((shapeLengthY/2),2.0))));//Finding the largest possible radius, rounding up
  for(int ii=0; ii<shapeLengthX; ++ii){
    for(int jj=0; jj<shapeLengthY; ++jj){
      croppedCentredBackground[ii+(sizeX-shapeLengthX)/2][jj+(sizeY-shapeLengthY)/2]=inputImage[ii+cropEdge[0]][jj+cropEdge[2]];
    }
  }
  /*******
   * 
   * Make rotated versions
   * (Make numberOfRotations (1) images, rotations of shape)
   * 
   *******/
  int nx;//New x coordinate for pixel after rotation
  int ny;//New y coordinate for pixel after rotation
  /**
   * Background, made to fit any orientation of shape.
   **/
  vector<vector<float>> largeBackground;
  largeBackground.resize(sizeX+2*boardSize);
  for(int imr=0; imr<sizeX+2*boardSize; ++imr){
    largeBackground[imr].resize(sizeY+2*boardSize);
  }
  //Set all pixels in background to 0
  for(int imr=0; imr<sizeX+2*boardSize; ++imr){
    for(int jmr=0; jmr<sizeY+2*boardSize; ++jmr){
      largeBackground[imr][jmr]=0;
    }
  }
  /**
   * Rotating the shape, and attaching it to the background
   **/
  vector<vector<vector<float>>> rotatedShapeVector;
  for(int imr=0; imr<numberOfRotations; ++imr){
    vector<vector<float>> shapeVector;
    shapeVector=largeBackground;
    for(int mmr=0; mmr<sizeX+2*boardSize; ++mmr){
      for(int nmr=0; nmr<sizeY+2*boardSize; ++nmr){
	nx=(mmr-sizeX/2+boardSize)*cos(imr*2*pi/numberOfRotations)+(nmr-sizeY/2+boardSize)*sin(imr*2*pi/numberOfRotations)+sizeX/2;
	ny=-(mmr-sizeX/2+boardSize)*sin(imr*2*pi/numberOfRotations)+(nmr-sizeY/2+boardSize)*cos(imr*2*pi/numberOfRotations)+sizeY/2;
	if(nx>0 && nx<sizeX && ny>0 && ny<sizeY && croppedCentredBackground[nx][ny]>0.5){
	  shapeVector[mmr][nmr]=1;
	}
      }
    }
    rotatedShapeVector.push_back(shapeVector);
  }
  referenceShapes.push_back(rotatedShapeVector);
  needToLearn[0]=0.f;
}

static InitClass init("ShapeDetector", &ShapeDetector::Create, "Source/UserModules/ShapeDetector/");


