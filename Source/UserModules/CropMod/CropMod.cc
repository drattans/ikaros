#include "CropMod.h"

using namespace ikaros;

void
CropMod::Init()
{
  input = GetInputMatrix("INPUT");//Input image
  sizeX = GetInputSizeX("INPUT");//Size of input image (x)
  sizeY = GetInputSizeY("INPUT");//Size of input image (y)
  NoC = GetIntValue("NoC", 1);//Maximum number of clusters
  centerIn = GetInputMatrix("CENTER_IN");//Coordinates of found clusters (x,y)
  imout = GetOutputMatrix("IMOUT");//Croped image with block

  cropsif = 0.05f;//Half the length of the side of the croped image (fraction of large image)
  cropsip = 50;//sizeX*cropsif;//Half the length of the side of the croped image (pixels)
}

void
CropMod::Tick()
{
  //centerIn[0][0]=centerIn[0][0]*640.f;
  //centerIn[1][0]=centerIn[1][0]*480.f;
  printf("X=%f, Y=%f\n", centerIn[0], centerIn[1]);
  NoC=1;//At least start with only one shape
  int i=1;
  //for(int i=0; i<NoC; ++i){//For all clusters
  //Crop out the object;
  if(centerIn[i][0]-cropsip >= 0 && centerIn[i][1]-cropsip >= 0 && centerIn[i][0]+cropsip < sizeX && centerIn[i][1]+cropsip < sizeY){
    for(int j=centerIn[i][0]-cropsip; j<centerIn[i][0]+cropsip; ++j){
      for(int k=centerIn[i][1]-cropsip; k<centerIn[i][1]+cropsip; ++k){
	//Place pixel (j,k) in output matrix (i)
	int a = j-(centerIn[i][0]-cropsip);
	int b = k-(centerIn[i][1]-cropsip);
	imout[a][b]=input[j][k];
      }
    }
  }
  else{//if it's close to the edge
    int sx=centerIn[i][0]-cropsip;
    int sy=centerIn[i][1]-cropsip;
    if(centerIn[i][0]-cropsip < 0){
      sx=0;
    }
    else if(centerIn[i][0]+cropsip >= sizeX){
      sx=sizeX-2*cropsip;
    }
    if(centerIn[i][1]-cropsip < 0){
      sy=0;
    }
    else if(centerIn[i][1]+cropsip >= sizeY){
      sy=sizeY-2*cropsip;
    }
    for(int j=sx; j<sx+2*cropsip; ++j){
      for(int k=sy; k<sy+2*cropsip; ++k){
	//Place pixel (j,k) in output matrix (i)
	int a = j-sx;
	int b = k-sy;
	imout[a][b]=input[j][k];
      }
    }
  }
}

static InitClass init("CropMod", &CropMod::Create, "Source/UserModules/CropMod/");
