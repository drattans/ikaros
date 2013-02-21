#ifndef SmMo_
#define SmMo_

#include "IKAROS.h"

class SmMo: public Module
{
 public:
  static Module * Create(Parameter * p) { return new SmMo(p); }

  float * cart_in;
  float * cart_out;
  float * servo_out;
  float * cupoa;
  float el;
  float y1;
  float y2;
  float k;
  float pi;
  float fe;
  float h;
  float abcx;
  float abcy;
  float ml;
  bool smooth;  

  SmMo(Parameter * p) : Module(p) {}
  virtual ~SmMo() {}

  void 		Init();
  void 		Tick();
};

#endif

