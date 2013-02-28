#include <math.h>
#include <vector>
using namespace std;

typedef vector<double> VecDoub;
typedef vector<VecDoub> MatDoub;

struct Base_interp{

  int n, mm, jsav, cor, dj;
  const double *xx, *yy;  
Base_interp(VecDoub &x, const double *y, int m)
: n(x.size()), mm(n), jsav(0), cor(0), xx(&x[0]), yy(y){
  dj = min(1, (int)pow((double)n, 0.25));
}

  double interp(double x){
    int jlo = cor ? hunt(x) : locate(x);
    return rawinterp(jlo, x);
  }

  int locate(const double x);
  int hunt(const double x);

  double virtual rawinterp(int jlo, double x) = 0;
};

int Base_interp::locate(const double x){
  int ju, jm, jl;
  if(n<2 || mm<2 ||mm>n){
    throw("locate size error");
  }
  bool ascnd = (xx[n-1] >= xx[0]);
  jl = 0;
  ju = n-1;
  while (ju-jl > 1){
    jm = (ju+jl) >> 1;
    if(x >= xx[jm] == ascnd){
      jl=jm;
    }
    else{
      ju=jm;
    }
  }
  cor = abs(jl-jsav) > dj ? 0 : 1;
  jsav = jl;
  return max(0,min(n-mm,jl-((mm-2)>>1)));
}

int Base_interp::hunt(const double x){
  int jl=jsav, jm, ju, inc=1;
  if(n<2 || mm<2 ||mm>n){
    throw("hunt size error");
  }
  bool ascnd = (xx[n-1] >= xx[0]);
  if(jl<0 || jl > n-1){
    jl = 0;
    ju = n-1;
  }
  else{
    if(x >= xx[jl] == ascnd){
      for(;;){
	ju=jl + inc;
	if(ju >= n-1){
	  ju = n-1;
	  break;
	}
	else if(x < xx[ju] == ascnd){
	  break;
	}
	else{
	  jl = ju;
	  inc += inc;
	}
      }
    }
    else{
      ju = jl;
      for(;;){
	jl = jl - inc;
	if(jl <= 0){
	  jl = 0;
	  break;
	}
	else if(x >= xx[jl] == ascnd){
	  break;
	}
	else{
	  ju = jl;
	  inc += inc;
	}
      }
    }
  }
  while(ju-jl > 1){
    jm = (ju+jl) >> 1;
    if(x >= xx[jm] == ascnd){
      jl = jm;
    }
    else{
      ju = jm;
    }
  }
  cor = abs(jl-jsav) > dj ? 0: 1;
  jsav = jl;
  return max(0,min(n-mm,jl-((mm-2)>>1)));
}

struct Spline_interp : Base_interp{
  VecDoub y2;
  
 Spline_interp(VecDoub &xv, VecDoub &yv, double yp1=1.e99, double ypn=1.e99)
   : Base_interp(xv, &yv[0], 2), y2(xv.size()){
    sety2(&xv[0], &yv[0], yp1, ypn);

  }
  
 Spline_interp(VecDoub &xv, const double *yv, double yp1=1.e99, double ypn=1.e99)
   : Base_interp(xv, yv, 2), y2(xv.size()){
    sety2(&xv[0], yv, yp1, ypn);
  }
  
  void sety2(const double *xv, const double *yv, double yp1, double ypn);
  double rawinterp(int jl, double xv);
};
  
void Spline_interp::sety2(const double *xv, const double *yv, double yp1, double ypn){
  int i, k;
  double p, qn, sig, un;
  int n = y2.size();
  VecDoub u(n-1);
  if(yp1 > 0.99e99){
    y2[0]=u[0]=0.0;
  }
  else{
    y2[0] = -0.5;
    u[0]=(3.0/(xv[1]-xv[0]))*((yv[1]-yv[0])/(xv[1]-xv[0])-yp1);
  }
  for(i=1; i<n-1; ++i){
    sig = (xv[i]-xv[i-1])/(xv[i+1]-xv[i-1]);
    p=sig*y2[i-1]+2.0;
    y2[i]=(sig-1.0)/p;
    u[i]=(yv[i+1]-yv[i])/(xv[i+1]-xv[i]) - (yv[i]-yv[i-1])/(xv[i]-xv[i-1]);
    u[i]=(6.0*u[i]/(xv[i+1]-xv[i-1])-sig*u[i-1])/p;
  }
  if(ypn > 0.99e99){
    qn=un=0.0;
  }
  else{
    qn=0.5;
    un=(3.0/(xv[n-1]-xv[n-2]))*(ypn-(yv[n-1]-yv[n-2])/(xv[n-1]-xv[n-2]));
  }
  y2[n-1]=(un-qn*u[n-2])/(qn*y2[n-2]+1.0);
  for(k=n-2; k>=0; k--){
    y2[k]=y2[k]*y2[k+1]+u[k];
  }
}

double Spline_interp::rawinterp(int jl, double x){
  int klo=jl, khi=jl+1;
  double y, h, b, a;
  h=xx[khi]-xx[klo];
  if(h==0.0){
    throw("Bad input to routine splint");
  }
  a = (xx[khi]-x)/h;
  b = (x-xx[klo])/h;
  y = a*yy[klo]+b*yy[khi]+((a*a*a-a)*y2[klo]+(b*b*b-b)*y2[khi])*(h*h)/6.0;
  return y;
}
/*
  struct Spline2D_interp{
  int m,n;
  const MatDoub &y;
  const VecDoub &x1;
  VecDoub yv;
  vector<Spline_interp*> srp;

  Spline2D_interp(VecDoub &x1v, VecDoub &x2v, MatDoub &ym)
  : m(x1v.size()), n(x2v.size()), y(ym), yv(m), x1(x1v), srp(m){
  for(int i=0; i<m; i++){
  srp[i] = new Spline_interp(x2v, &y[i][0]);
  }
  }

  ~Spline2D_interp(){
  for(int i=0; i<m; i++){
  delete srp[i];
  }
  }

  double interp(double x1p, double x2p){
  for(int i=0; i<m; i++){
  yv[i] = (*srp[i]).interp(x2p);
  }
  Spline_interp scol(x1, yv);
  return scol.interp(x1p);
  }

  };
*/
