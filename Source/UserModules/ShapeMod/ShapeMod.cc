#include "ShapeMod.h"
#include "math.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <ctime>

using namespace ikaros;
using namespace std;

void
ShapeMod::Init()
{
  srand(std::time(NULL));
  iminc=GetInputMatrix("IMINC");
  //imine=GetInputMatrix("INIME");//Edge, not in use yet
  //siX=GetInputSizeX("INIMC");//Should be 100
  //siY=GetInputSizeY("INIMC");//Should be 100
  siX=100;
  siY=100;
  //err=GetInputArray("ERR");//Look for new shape?(1==true)
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
  cin[0]=cin[0]*640.f;
  cin[1]=cin[1]*480.f;
  if(proSh.size()==0){// || err<0.5){
    float av=0;
    for(int m=0; m<siX; ++m){//proSh[i].size()=150
      for(int n=0; n<siY; ++n){
	av+=iminc[m][n];
      }
    }
    av/=siX*siY;
    printf("AV=%f\n", av);
    if(av>0.05){
      makeShape();
    }
  }
  if(proSh.size()>0){
    findShape();
    //cout[0]+=cin[0];
    //cout[1]+=cin[1];
    cout[0]/=640.f;
    cout[1]/=480.f;
  }
  else{
    cout[0]=cin[0]/640.f;
    cout[1]=cin[1]/480.f;
  }
  //Is largest radious possible to find?
  //Yes, using information from the croping process when the shape was created
}

void
ShapeMod::findShape()
{
  printf("Finding shape!\n");
  int countM=0;
  //For each shape:
  for(int i=0; i<proSh.size(); ++i){
    printf("Shape number %i!\n", i);
    /*//Brute force
    //For each orientation
    for(int l=0; l<noR; ++l){
      printf("Rotation number %i!\n", l);
      //For each x-position:
      for(int j=-35; j<-15; ++j){
	//For each y-position:
	for(int k=-35; k<-15; ++k){
	  //Check similarity
	  int count=0;
	  count=eval(j, k, l, i);
	  if(count>countM){
	    countM=count;//Agreeing pixels
	    cout[0]=j+proSh[i][l].size()/2 + cin[0]-siX/2;//Center in x
	    cout[1]=k+proSh[i][l].size()/2 + cin[1]-siY/2;//Center in y
	    shapeIn[0]=i;//Shape index
	    shapeDir[0]=l;//Shape direction
	    //printf("(J,K)=(%i,%i) ([%d:%d])\n", j, k, -siX/2, 0);
	    //printf("Count!!: %i\n", count);
	  }
	}
      }
    }
    //*/
    /*//Greedy
    int l=0;
    bool fin=false;
    int lc=0;
    int j=-25;
    int k=-25;
    int count [4];
    int cold=eval(j, k, l, i);
    int wi=4;
    int rad=1;
    while(fin==false){
      ++lc;
      //fin=false;
      wi=4;
      rad=rand()%10+1;
      count[0]=eval(j-rad, k, l, i);
      count[1]=eval(j+rad, k, l, i);
      count[2]=eval(j, k-rad, l, i);
      count[3]=eval(j, k+rad, l, i);
      for(int floo=0; floo<4; ++floo){
	if(count[floo]>cold){
	  wi=floo;
	  cold=count[wi];
	}
      }
      //printf("Winner!!: %i\n", wi);
      if(wi==0){
	j-=rad;
	//cold=count[wi];
      }
      else if(wi==1){
	j+=rad;
	//cold=count[wi];
      }
      else if(wi==2){
	k-=rad;
	//cold=count[wi];
      }
      else if(wi==3){
	k+=rad;
	//cold=count[wi];
      }
      else if(lc>10){
	fin=true;
      }
      if(lc>100){
	fin=true;
      }
    }
    cout[0]=j+proSh[i][l].size()/2 + cin[0]-siX/2;//Center in x
    cout[1]=k+proSh[i][l].size()/2 + cin[1]-siY/2;//Center in y
    shapeIn[0]=i;//Shape index
    shapeDir[0]=l;//Shape direction
    printf("(J,K)=(%i,%i) ([%d:%d])\n", j, k, -siX/2, 0);
    printf("Count!!: %i\n", cold);
    printf("Loops!!: %i\n", lc);
    //*/
    //*//Simulated annealing
    int l=0;
    int j=-25;
    int k=-25;
    int jo=-25;
    int ko=-25;
    int count=0;
    int cold=0;
    cold=eval(j, k, l, i);
    for(int anva=0; anva<300; ++anva){
      j=jo+rand()%100/(anva+1)+rand()%2;
      k=ko+rand()%100/(anva+1)+rand()%2;
      l=rand()%noR;
      //j=jo+rand()%(100-anva)+rand()%2;
      //k=ko+rand()%(100-anva)+rand()%2;
      count=eval(j, k, l, i);
      if(count>cold){
	jo=j;
	ko=k;
	cold=count;
	printf("ANVA!!: %i\n", anva);
      }
    }
    cout[0]=jo+proSh[i][l].size()/2 + cin[0]-siX/2;//Center in x
    cout[1]=ko+proSh[i][l].size()/2 + cin[1]-siY/2;//Center in y
    shapeIn[0]=i;//Shape index
    shapeDir[0]=l;//Shape direction
    printf("(J,K)=(%i,%i) ([%d:%d])\n", jo, ko, l, 0);
    printf("Count!!: %i\n", cold);
    //*/
  }
}

int
ShapeMod::eval(int je, int ke, int le, int ie)
{
  int counte=0;
  for(int m=0; m<proSh[ie][le].size(); ++m){//proSh[i].size()=150
    for(int n=0; n<proSh[ie][le].size(); ++n){
      if(je+m>0 && siX>je+m && ke+n>0 && siY>ke+n){
        if(iminc[je+m][ke+n]>0.5 && proSh[ie][le][m][n]>0.5){
	  ++counte;
        }
      }
    }
  }
  /*
  if(counte>0){
    printf("Count!!: %i\n", counte);
  }
  //*/
  return counte;
}

void
ShapeMod::makeShape()
{
  printf("Making shape!\n");
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
      if(iminc[ics][jcs]>0.5){
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
  //int flag = 0;
  //printf("W=%i, H=%i\n", wid, hei);
  //int hmp = 0;
  for(int ii=0; ii<wid; ++ii){
    for(int jj=0; jj<hei; ++jj){
      //++flag;
      //printf("Flag=%i, Val=%f\n", flag, iminc[ii+cred[0]][jj+cred[2]]);
      csbg[ii+(siX-wid)/2][jj+(siY-hei)/2]=iminc[ii+cred[0]][jj+cred[2]];
      //if(iminc[ii+cred[0]][jj+cred[2]]==1){
      //++hmp;
      //}
    }
  }
  //printf("*~~~~~\n* HMP: %i!\n*~~~~~\n", hmp);
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
    for(int mmr=0; mmr<siX; ++mmr){
      for(int nmr=0; nmr<siY; ++nmr){
	//printf("?=%f\n", csbg[mmr][nmr]);
	if(csbg[mmr][nmr]>0.5){
	  nx=(mmr-50)*cos(imr*2*pi/noR)-(nmr-50)*sin(imr*2*pi/noR)+75;
	  ny=(mmr-50)*sin(imr*2*pi/noR)+(nmr-50)*cos(imr*2*pi/noR)+75;
	  si[nx][ny]=1;
	  //printf("?=%f, nx=%i, ny=%i\n", si[nx][ny], nx, ny);
	}
      }
    }
    siv.push_back(si);
  }
  proSh.push_back(siv);
  //printShape();//When a new shape is created, the shapes are printed to a file for inspection
}

void
ShapeMod::printShape()
{
  ofstream sho;
  sho.open ("shape.txt");
  //For each shape:
  //for(int i=0; i<proSh.size(); ++i){
  //For each orientation
  //for(int l=0; l<noR; ++l){
  //For each x-position:
  //printf("(%i, %i, %i, %i)\n", proSh.size(), proSh[0].size(), proSh[0][0].size(), proSh[0][0][0].size());
  for(int mn=0; mn<noR; ++mn){
  int hmpp = 0;
  for(int m=0; m<150; ++m){//proSh[i].size()=150
    for(int n=0; n<150; ++n){
      if(proSh[0][mn][m][n]==1){
	++hmpp;
      }
      sho << proSh[0][mn][m][n];
    }
    sho << "\n";
  }
  printf("*~~~~~\n* HMPP: %i!\n*~~~~~\n", hmpp);
  }
  //}
  //}
}

static InitClass init("ShapeMod", &ShapeMod::Create, "Source/UserModules/ShapeMod/");


