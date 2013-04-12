#include "EdgeMod.h"

using namespace ikaros;

void
EdgeMod::Init()
{
  inputsize_x	= GetInputSizeX("INPUT");
  inputsize_y	= GetInputSizeY("INPUT");

  outputsize_x = GetOutputSizeX("OUTPUT");
  outputsize_y = GetOutputSizeY("OUTPUT");

  input = GetInputMatrix("INPUT");
  output = GetOutputMatrix("OUTPUT");
}



void
EdgeMod::Tick()
{
  for (int j =0; j<inputsize_y-1; j++){
    for (int i=0; i<inputsize_x-1; i++){
      float dx = input[j][i+1] - input[j+1][i];
      float dy = input[j][i] - input[j+1][i+1];
      output[j][i] = sqrt(dx*dx+dy*dy);
    }
  }
}



static InitClass init("EdgeMod", &EdgeMod::Create, "Source/UserModules/EdgeMod/");


