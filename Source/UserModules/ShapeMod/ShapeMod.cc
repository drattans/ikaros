#include "ShapeMod.h"
#include "math.h"

using namespace ikaros;

void
ShapeMod::Init()
{
  iminc=GetInputMatrix("INIMC");
  imine=GetInputMatrix("INIME");//Edge, not in use yet
  siX=GetInputSizeX("INIMC");
  siY=GetInputSizeY("INIMC");
  err=GetInputArray("ERR");//Look for new shape?(1==true)
  cin=GetInputArray("CIN");//Center of input
  nor=10;//Number of rotations
  pi=atan(1.f)*4.f;//Pi
  cout=GetOutputArray("COUT");
  shapeIn=GetOutputArray("INDEX");
  shapeDir=GetOutputArray("DIRECTION");
}

void
ShapeMod::Tick()
{
  if(proSh.size()==0 || err<0.5){
    cropShape();
    makeRotations();
  }
  findShape();
  cout[0]+=cin[0];
  cout[1]+=cin[1];

  //Is largest radious possible to find?
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
      for(int j=0; j<siX-proSh[i].size(); ++j){
	//For each y-position:
	for(int k=0; k<siY-proSh[i].size(); ++k){
	  //Check similarity
	  int count;
	  for(int m=0; m<proSh[i].size(); ++m){
	    for(int n=0; n<proSh[i].size(); ++n){
	      if(iminc[j][k]==1 && proSh[i][l][m][n]==1){
		++count;
	      }
	    }
	  }
	  if(count>countM){
	    countM=count;//Agreeing pixels
	    cout[0]=j+proSh[i][l].size()/2 + cin[0]-siX/2;//Center in x
	    cout[1]=k+proSh[i][l][j].size()/2 + cin[1]-siY/2;//Center in y
	    shapeIn=i;//Shape index
	    shapeDir=l;//Shape direction
	  }
	}
      }
    }
  }
}

void
ShapeMod::cropShape()
{
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
  //Maybe not possible if fixed size is needed
  //Crop, then place in middle of standard sized frame?
  //for all collumns:
  //Look for first and last non-empty collumn
  int cred [4]={0,0,siY,0};//Edges of croped image(x1,x2,y1,y2)
  for(int ics=0; ics<siX; ++ics){
    //for all row:
    //Look for first and last non-empty row
    for(int jcs=0; jcs<siY; ++jcs){
      if(inimc[ics][jcs]==1){
	if(cred[0]==0){
	  cred[0]=ics;
	}
	else{
	  cred[1]=ics;
	}
	if(cred[2]>jsc){
	  cred[2]=jcs;
	}
	if(cred[3]<jcs){
	  cred[3]=jcs;
	}
      }   
    }
  }
  //add the croped part to vectorvector
  int wid = cred[1]-cred[0];
  int hei = cred[3]-cred[2];
  for(int ii; ii<wid; ++ii){
    for(int jj; jj<hei; ++jj){
      csbg[ii+(siX-wid)/2][jj+(siY-hei)/2]=inimc[ii+cred[0]][jj+cred[2]];
    }
  }
}

void
ShapeMod::makeRotations()
{//Make noR (10) images, rotations of shape
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
    for(int mmr=0; mmr<siX; ++mmr){
      for(int nmr=0; nmr<siY; ++nmr){
	if(iminc[mmr][nmr]==1){
	  nx=sqrt(pow((mmr-siX/2),2)+pow((nmr-siY/2),2))*cos(imr*2*pi/noR)+mmr+siX*0.25;
	  ny=sqrt(pow((mmr-siX/2),2)+pow((nmr-siY/2),2))*sin(imr*2*pi/noR)+nmr+siY*0.25;
	  si[nx][ny]=1;
	}
      }
    }
    siv.push_back(si);
  }
  proSh.push_back(siv);
}


static InitClass init("ShapeMod", &ShapeMod::Create, "Source/UserModules/ShapeMod/");


