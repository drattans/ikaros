#include "ShapeMod.h"
#include "math.h"
#include <vector>

using namespace ikaros;
using namespace std;

void
ShapeMod::Init()
{
  iminc=GetInputMatrix("INIMC");
  imine=GetInputMatrix("INIME");//Edge, not in use yet
  siX=GetInputSizeX("INIMC");//Should be 100
  siY=GetInputSizeY("INIMC");//Should be 100
  //siX=100;
  //siY=100;
  err=GetInputArray("ERR");//Look for new shape?(1==true)
  cin=GetInputArray("CIN");//Center of input
  noR=10;//Number of rotations
  pi=atan(1.f)*4.f;//Pi
  cout=GetOutputArray("COUT");
  shapeIn=GetOutputArray("INDEX");
  shapeDir=GetOutputArray("DIRECTION");
  //proSh;
}

void
ShapeMod::Tick()
{
  if(proSh.size()==0){// || err<0.5){
    makeShape();
  }
  findShape();
  cout[0]+=cin[0];
  cout[1]+=cin[1];

  //Is largest radious possible to find?
  //Yes, using information from the croping process when the shape was created
}

void
ShapeMod::findShape()
{
  int countM;
  //For each shape:
  for(int i=0; i<proSh.size(); ++i){
    //For each orientation
    for(int l=0; l<noR; ++l){
      //For each x-position:
      for(int j=-3*siX/4; j<siX/2; ++j){
	//For each y-position:
	for(int k=-3*siY/4; k<siY/2; ++k){
	  //Check similarity
	  int count;
	  for(int m=0; m<proSh[i].size(); ++m){//proSh[i].size()=150
	    for(int n=0; n<proSh[i].size(); ++n){
	      if(j+m>0 && siX<j+m && k+n>0 && siY<k+n){
		if(iminc[j+m][k+n]==1 && proSh[i][l][m][n]==1){
		  ++count;
		}
	      }
	    }
	  }
	  if(count>countM){
	    countM=count;//Agreeing pixels
	    cout[0]=j+proSh[i][l].size()/2 + cin[0]-siX/2;//Center in x
	    cout[1]=k+proSh[i][l][j].size()/2 + cin[1]-siY/2;//Center in y
	    shapeIn[0]=i;//Shape index
	    shapeDir[0]=l;//Shape direction
	  }
	}
      }
    }
  }
}

void
ShapeMod::makeShape()
{
  /*******
   * 
   * Crop the image
   * 
   *******/
  vector<vector<float>> csbg;
  csbg.resize(siX);
  for(int imr=0; imr<siX; ++imr){
    csbg[imr].resize(siY);
  }
  //Set all pixels in background to 0
  for(int imr=0; imr<siX; ++imr){
    for(int jmr=0; jmr<siY; ++jmr){
      csbg[imr][jmr]=0;
    }
  }
  //for all collumns:
  //Look for first and last non-empty collumn
  int cred [4]={0,0,siY,0};//Edges of croped image(x1,x2,y1,y2)
  for(int ics=0; ics<siX; ++ics){
    //for all row:
    //Look for first and last non-empty row
    for(int jcs=0; jcs<siY; ++jcs){
      if(iminc[ics][jcs]==1){
	if(cred[0]==0){
	  cred[0]=ics;
	}
	else{
	  cred[1]=ics;
	}
	if(cred[2]>jcs){
	  cred[2]=jcs;
	}
	if(cred[3]<jcs){
	  cred[3]=jcs;
	}
      } 
    }
  }
  //add the croped part to the center of a vectorvector
  int wid = cred[1]-cred[0];//Width of croped image
  int hei = cred[3]-cred[2];//Height of croped image
  for(int ii; ii<wid; ++ii){
    for(int jj; jj<hei; ++jj){
      csbg[ii+(siX-wid)/2][jj+(siY-hei)/2]=iminc[ii+cred[0]][jj+cred[2]];
    }
  }
  /*******
   * 
   * Make rotated versions
   * (Make noR (10) images, rotations of shape)
   * 
   *******/
  int nx;//New x coordinate for pixel
  int ny;//New y coordinate for pixel
  /**
   * Background, made to fit any orientation of shape.
   **/
  vector<vector<float>> sbg;
  sbg.resize(siX*1.5);
  for(int imr=0; imr<siX*1.5; ++imr){
    sbg[imr].resize(siY*1.5);
  }
  //Set all pixels in background to 0
  for(int imr=0; imr<siX*1.5; ++imr){
    for(int jmr=0; jmr<siY*1.5; ++jmr){
      sbg[imr][jmr]=0;
    }
  }
  /**
   * Rotating the shape, and attaching it to the background
   **/
  vector<vector<vector<float>>> siv;
  for(int imr=0; imr<noR; ++imr){
    vector<vector<float>> si;
    si=sbg;
    for(int mmr=0; mmr<wid; ++mmr){
      for(int nmr=0; nmr<hei; ++nmr){
	if(csbg[mmr][nmr]==1){
	  nx=sqrt(pow((double)(mmr-wid/2),2)+pow((double)(nmr-hei/2),2))*cos(imr*2*pi/noR)+mmr+siX*0.25;
	  ny=sqrt(pow((double)(mmr-wid/2),2)+pow((double)(nmr-hei/2),2))*sin(imr*2*pi/noR)+nmr+siY*0.25;
	  si[nx][ny]=1;
	}
      }
    }
    siv.push_back(si);
  }
  proSh.push_back(siv);
  printShape();//When a new shape is created, the shapes are printed to a file for inspection
}

void
ShapeMod::printShape()
{
  //for-loop through all the pixels, and print them to a textfile
}

static InitClass init("ShapeMod", &ShapeMod::Create, "Source/UserModules/ShapeMod/");


