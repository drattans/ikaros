#ifndef CropMod_
#define CropMod_

#include "IKAROS.h"

class CropMod: public Module
{
 public:
  static Module * Create(Parameter * p) { return new CropMod(p); }

  float ** input;
  float ** centerIn;
  float ** imout;
  //float ** centerOut;

  //float * radious;
  //float * shor;

  float cropsif;

  int cropsip;
  int NoC;
  int sizeX;
  int sizeY;

 CropMod(Parameter * p) : Module(p) {}
  virtual ~CropMod() {}

  void 		Init();
  void 		Tick();
};

#endif

