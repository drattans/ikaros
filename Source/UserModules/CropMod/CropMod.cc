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
  //centerOut = GetOutputMatrix("CENTER_OUT");//Coordinates of modified clusters (x,y)
  //radious = GetOutputArray("RADIOUS");//The largest radious of the object
  //shor = GetOutputArray("SHOR");//Orientation of shape

  cropsif = 0.05f;//Half the length of the side of the croped image (fraction of large image)
  cropsip = 50;//sizeX*cropsif;//Half the length of the side of the croped image (pixels)
}

void
CropMod::Tick()
{
  NoC=1;//At least start with only one shape
  int i=1;
  //for(int i=0; i<NoC; ++i){//For all clusters
  //Crop out the object;
  if(centerIn[i][0]-cropsip > 0 && centerIn[i][1]-cropsip > 0 && centerIn[i][0]+cropsip < sizeX && centerIn[i][1]+cropsip < sizeY){
    for(int j=centerIn[i][0]-cropsip; j<centerIn[i][0]+cropsip; ++j){
      for(int k=centerIn[i][1]-cropsip; k<centerIn[i][1]+cropsip; ++k){
	//Place pixel (j,k) in output matrix (i)
	int a = j-(centerIn[i][0]-cropsip);
	int b = k-(centerIn[i][1]-cropsip);
	imout[a][b]=input[j][k];
      }
    }
  }
}

static InitClass init("CropMod", &CropMod::Create, "Source/UserModules/CropMod/");
