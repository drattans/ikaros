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
  imout=GetOutputMatrix("IMOUT");
  //imine=GetInputMatrix("INIME");//Edge, not in use yet
  //siX=GetInputSizeX("INIMC");//Should be 100
  //siY=GetInputSizeY("INIMC");//Should be 100
  nodl = GetFloatValue("NODL", 1);//Number of dilation loops  
  norou = GetOutputArray("NOROU");
  siX=100;
  siY=100;
  err=GetInputArray("ERR");//Need new shape?(1==true)
  cin=GetInputArray("CIN");//Center of input
  noR=1;//Number of rotations
  pi=atan(1.f)*4.f;//Pi
  cout=GetOutputArray("COUT");
  shapeIn=GetOutputArray("INDEX");
  shapeDir=GetOutputArray("DIRECTION");
  //proSh;
  //mix=-25;
  //max=-25;
  //miy=-25;
  //may=-25;
}

void
ShapeMod::Tick()
{
  cin[0]=cin[0]*640.f;
  cin[1]=cin[1]*480.f;
  //* Filling gaps in input
  if(nodl>0){
    vector<vector<float>> iminct;// = new float[siX][siY];
    iminct.resize(siX);
    for(int imr=0; imr<siX; ++imr){
      iminct[imr].resize(siY);
    }
    for(int k=0; k<nodl; ++k){
      for (int j =1; j<siX-1; j++){
	for (int i=1; i<siY-1; i++){
	  float t = iminc[j][i];
	  for (int jj=-1; jj<2; jj++){
	    for (int ii=-1; ii<2; ii++){
	      if (iminc[j+jj][i+ii] > t){
		t = iminc[j+jj][i+ii];
	      }
	    }
	  }
	  iminct[j-1][i-1] = t;
	}
      }
      for (int j =0; j<siX; j++){
	for (int i=0; i<siY; i++){
	  iminc[j][i]=iminct[j][i];
	}
      }
    }
  }
  //*/
  if(proSh.size()==0 || err[0]>0.5){
    float av=0;
    for(int m=0; m<siX; ++m){//proSh[i].size()=150
      for(int n=0; n<siY; ++n){
	av+=iminc[m][n];
      }
    }
    av/=siX*siY;
    //printf("AV=%f\n", av);
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
  int countM=0;
  //For each shape:
  for(int i=0; i<proSh.size(); ++i){
    //*//Brute force
    //For each orientation
    int ko;
    int jo;
    int lo=0;
    for(int l=0; l<noR; ++l){
      //printf("Rotation number %i!\n", l);
      //For each x-position:
      for(int j=-35; j<-15; ++j){
	//For each y-position:
	for(int k=-35; k<-15; ++k){
	  //Check similarity
	  int count=0;
	  count=eval(j, k, l, i);
	  if(count>countM){
	    countM=count;//Agreeing pixels
	    cout[0]=cin[0]+(k+25);//j+proSh[i][l].size()/2 + cin[0]-siX/2;//Center in x
	    cout[1]=cin[1]+(j+25);//k+proSh[i][l].size()/2 + cin[1]-siY/2;//Center in y
	    shapeIn[0]=i;//Shape index
	    shapeDir[0]=l;//Shape direction
	    norou[0]=proSh[i].size();
	    ko=k;
	    jo=j;
	    lo=l;
	  }
	}
      }
    }
    int l=lo;
    /*
    if(mix>jo){
      mix=jo;
    }
    if(max<jo){
      max=jo;
    }
    if(miy>ko){
      miy=ko;
    }
    if(may<ko){
      may=ko;
    }
    printf("(X,Y)=([%i,%i],[%i,%i])\n", mix, max, miy, may);
    */
    //printf("Count!!: %i, %i, %i\n", l, lo, i);
    //*/
    /*//Greedy
    int ko;
    int jo;
    int lo;
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
    ko=k;
    jo=j;
    lo=l;
    cout[0]=cin[0]+(k+25);
    cout[1]=cin[1]+(j+25);
    //cout[0]=j+proSh[i][l].size()/2 + cin[0]-siX/2;//Center in x
    //cout[1]=k+proSh[i][l].size()/2 + cin[1]-siY/2;//Center in y
    shapeIn[0]=i;//Shape index
    shapeDir[0]=l;//Shape direction
    //printf("(J,K)=(%i,%i) ([%d:%d])\n", j, k, -siX/2, 0);
    //printf("Count!!: %i\n", cold);
    //printf("Loops!!: %i\n", lc);
    //*/
    /*//Simulated annealing
    int l=0;
    int j=-25;
    int k=-25;
    int jo=-25;
    int ko=-25;
    int count=0;
    int cold=0;
    cold=eval(j, k, l, i);
    for(int anva=0; anva<100; ++anva){
      j=jo+rand()%100/(anva+1)+rand()%2;
      k=ko+rand()%100/(anva+1)+rand()%2;
      l=rand()%noR;
      //j=jo+rand()%(1600-anva)+rand()%2;
      //k=ko+rand()%(1600-anva)+rand()%2;
      count=eval(j, k, l, i);
      if(count>cold){
	jo=j;
	ko=k;
	cold=count;
	printf("ANVA!!: %i\n", anva);
      }
    }
    cout[0]=cin[0]+(k+25);
    cout[1]=cin[1]+(j+25);
    //cout[0]=(jo+25)-25+proSh[i][l].size()/2 + cin[0]-siX/2;//Center in x
    //cout[1]=(ko+25)-25+proSh[i][l].size()/2 + cin[1]-siY/2;//Center in y
    shapeIn[0]=i;//Shape index
    shapeDir[0]=l;//Shape direction
    //printf("(J,K)=(%i,%i) ([%d:%d])\n", jo, ko, l, 0);
    //printf("Count!!: %i\n", cold);
    //*/
    for(int m=0; m<proSh[i][l].size(); ++m){//proSh[i].size()=150
      for(int n=0; n<proSh[i][l].size(); ++n){
	if(jo+m>0 && siX>jo+m && ko+n>0 && siY>ko+n){
	  //if(iminc[jo+m][ko+n]>0.5 && proSh[i][l][m][n]>0.5){
	  imout[m][n]=(iminc[jo+m][ko+n]+proSh[i][l][m][n])/2;
	  //}
	}
      }
    }
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
  return counte;
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
    //*
    for(int mmr=0; mmr<150; ++mmr){
      for(int nmr=0; nmr<150; ++nmr){
	//printf("?=%f\n", csbg[mmr][nmr]);
	nx=(mmr-75)*cos(imr*2*pi/noR)+(nmr-75)*sin(imr*2*pi/noR)+50;
	ny=-(mmr-75)*sin(imr*2*pi/noR)+(nmr-75)*cos(imr*2*pi/noR)+50;
	if(nx>0 && nx<100 && ny>0 && ny<100 && csbg[nx][ny]>0.5){
	  si[mmr][nmr]=1;
	  //printf("?=%f, nx=%i, ny=%i\n", si[nx][ny], nx, ny);
	}
      }
    }
    //*/
    /*
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
    //*/
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
  //printf("*~~~~~\n* HMPP: %i!\n*~~~~~\n", hmpp);
  }
  //}
  //}
}

static InitClass init("ShapeMod", &ShapeMod::Create, "Source/UserModules/ShapeMod/");


