#ifndef ShapeMod_
#define ShapeMod_

#include "IKAROS.h"
#include <vector>

class ShapeMod: public Module
{
public:
    static Module * Create(Parameter * p) { return new ShapeMod(p); }

    float ** iminc;
    float ** imine;

    int siX;
    int siY;
    int noR;

    float pi;

    float * err;
    float * cin;
    float * cout;
    float * shapeIn;
    float * shapeDir;

    std::vector<std::vector<float>> shape;
    std::vector<std::vector<std::vector<std::vector<float>>>> proSh; 

    ShapeMod(Parameter * p) : Module(p) {}
    virtual ~ShapeMod() {}

    void Init();
    void Tick();
    void findShape();
    void makeShape();
    void printShape();
};

#endif

