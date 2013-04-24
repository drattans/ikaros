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
  centerIn[0][0]=centerIn[0][0]*sizeX;
  centerIn[0][1]=centerIn[0][1]*sizeY;
  //printf("X=%f, Y=%f\n", centerIn[0][0], centerIn[0][1]);
  NoC=1;//At least start with only one shape
  int i=0;
  //for(int i=0; i<NoC; ++i){//For all clusters
  //Crop out the object;
  if(centerIn[i][0]-cropsip >= 0 && centerIn[i][1]-cropsip >= 0 && centerIn[i][0]+cropsip < sizeX && centerIn[i][1]+cropsip < sizeY){
    //printf("CASE 1!\n");
    for(int j=centerIn[i][0]-cropsip; j<centerIn[i][0]+cropsip; ++j){
      for(int k=centerIn[i][1]-cropsip; k<centerIn[i][1]+cropsip; ++k){
	//Place pixel (j,k) in output matrix (i)
	int a = j-(centerIn[i][0]-cropsip);
	int b = k-(centerIn[i][1]-cropsip);
	//printf("(A,B)=(%i,%i)\n", a, b);
	//printf("(X,Y)=(%i,%i)\n", sizeX, sizeY);
	//printf("(J,K)=(%i,%i)\n", j, k);
	//printf("(X,Y)=(%f,%f)\n", centerIn[i][0]+cropsip, centerIn[i][1]+cropsip);
	imout[b][a]=input[k][j];
      }
    }
  }
  else{//if it's close to the edge
    //printf("CASE 2!\n");
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
	imout[b][a]=input[k][j];
      }
    }
  }
}

static InitClass init("CropMod", &CropMod::Create, "Source/UserModules/CropMod/");
