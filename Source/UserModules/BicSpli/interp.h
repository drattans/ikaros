#include <math.h>
#include <vector>
using namespace std;

typedef vector<double> VecDoub;
typedef vector<VecDoub> MatDoub;

struct Spline_interp{
  VecDoub k;
  VecDoub xxI;
  VecDoub yyI;
  Spline_interp(VecDoub &xv, VecDoub &yv){
    double ki;
    for(int i=0; i<xv.size()-1; ++i){
      ki = (yv.at(i+1)-yv.at(i))/(xv.at(i+1)-xv.at(i));
      k.push_back(ki);
    }
    xxI=xv;
    yyI=yv;
  }

  double interp(double x){
    int t=xxI.size()-1;
    double ret;
    for(int i=0; i<xxI.size(); ++i){
      if(x<xxI.at(i)){
	t=i-1;
	break;
      }
    }
    if(t<0){
      ret = yyI.at(0) - k.at(0)*(xxI.at(0)-x);
    }
    else if(t==xxI.size()-1){
      ret = yyI.at(t) + k.at(t-1)*(x-xxI.at(t));
    }
    else{
      ret = yyI.at(t) + k.at(t)*(x-xxI.at(t));
    }    
    return ret;
  }
};
