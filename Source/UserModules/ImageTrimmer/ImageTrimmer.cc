//
//	ImageTrimmer.cc		This file is a part of the IKAROS project
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


#include "ImageTrimmer.h"

using namespace ikaros;

void
ImageTrimmer::Init()
{
  inputImage = GetInputMatrix("INPUT_IMAGE");//Input image
  sizeX = GetInputSizeX("INPUT_IMAGE");//Size of input image (x)
  sizeY = GetInputSizeY("INPUT_IMAGE");//Size of input image (y)
  numberOfClusters = GetIntValue("NUMBER_OF_CLUSTERS", 1);//Maximum number of clusters
  inputCenters = GetInputMatrix("INPUT_CENTERS");//Coordinates to the centers of the found clusters ((x1,y1), (x2,y2), ...)

  outputImage = GetOutputMatrix("OUTPUT_IMAGE");//Output image, centered around one of the clusters (100 x 100)

  halfSideLength = 50;//Half the length of the side of the croped image (pixels)
}


void
ImageTrimmer::Tick()
{
  int startX;//First pixel in the trimmed image, x
  int startY;//First pixel in the trimmed image, y
  numberOfClusters=1;//For more than one cluster, an array of images is needed to be sent as output
  for(int i=0; i<numberOfClusters; ++i){//Changing coordinates from fraction of image size to pixel values
    inputCenters[i][0]=inputCenters[i][0]*sizeX;
    inputCenters[i][1]=inputCenters[i][1]*sizeY;
  }
  for(int i=0; i<numberOfClusters; ++i){//For all clusters, make an output image
    startX=inputCenters[i][0]-halfSideLength;
    startY=inputCenters[i][1]-halfSideLength;
    if(startX<0){
      startX=0;
    }
    else if(inputCenters[i][0]+halfSideLength >= sizeX){
      startX=sizeX-2*halfSideLength;
    }
    if(startY<0){
      startY=0;
    }
    else if(inputCenters[i][1]+halfSideLength >= sizeY){
      startY=sizeY-2*halfSideLength;
    }
    for(int j=startX; j<startX+2*halfSideLength; ++j){
      for(int k=startY; k<startY+2*halfSideLength; ++k){
	//Place pixel (j,k) in the output matrix
	outputImage[j-startX][k-startY]=inputImage[j][k];
      }
    }
  }
}

static InitClass init("ImageTrimmer", &ImageTrimmer::Create, "Source/UserModules/ImageTrimmer/");


