#ifndef MoCoT_
#define MoCoT_

#include "IKAROS.h"

class MoCoT: public Module
{
 public:
  static Module * Create(Parameter * p) { return new MoCoT(p); }

  float * cart_in;
  float * servo_out;
  float y1;
  float y2;
  float k;
  float pi;
  float fe;
  float h;
  float abcx;
  float abcy;
  
  MoCoT(Parameter * p) : Module(p) {}
  virtual ~MoCoT() {}

  void 		Init();
  void 		Tick();
};

#endif

