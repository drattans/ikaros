#ifndef EdgeMod_
#define EdgeMod_

#include "IKAROS.h"

class EdgeMod: public Module
{
public:
    static Module * Create(Parameter * p) { return new EdgeMod(p); }

    EdgeMod(Parameter * p) : Module(p) {}
    virtual ~EdgeMod() {}

    void SetSizes();
    void Init();
    void Tick();

    int	inputsize_x;
    int	inputsize_y;

    int	outputsize_x;
    int	outputsize_y;

    float ** input;
    float ** output;
};

#endif

