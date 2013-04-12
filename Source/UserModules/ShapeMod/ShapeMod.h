#ifndef ShapeMod_
#define ShapeMod_

#include "IKAROS.h"

class ShapeMod: public Module
{
public:
    static Module * Create(Parameter * p) { return new ShapeMod(p); }

    vector<vector<float>> shape;

    vector<vector<vector<float>>> ProSh; 

    ShapeMod(Parameter * p) : Module(p) {}
    virtual ~ShapeMod() {}

    void Init();
    void Tick();
    void findShape();
};

#endif

